using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

using Rhino;
using Rhino.Geometry;

using Grasshopper.Kernel;

namespace MyPlugin.Components
{
    // -----------------------------------------------------------------------
    //  Curve Metaballer with sun modulation.
    //  Volumises curves into a mesh; radius and blend are scaled by a sampled
    //  sun value (sunny areas = thinner, more melted at junctions). Builds a
    //  sparse SDF grid and polygonises it with marching cubes.
    // -----------------------------------------------------------------------
    public class CurveMetaballerComponent : GH_Component
    {
        public CurveMetaballerComponent()
          : base("Curve Metaballer", "Metaball",
                 "Volumises curves into a mesh via a smooth-min SDF, with radius/blend " +
                 "modulated by a sampled sun field. Polygonised with marching cubes.",
                 "Geoboid", "Geoboid")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curves", "C", "Curves to volumise.", GH_ParamAccess.list);
            pManager.AddPointParameter("Sun Points", "SP", "Sun sample points (optional).", GH_ParamAccess.list);
            pManager.AddNumberParameter("Sun Values", "SV", "Sun values parallel to Sun Points (optional).", GH_ParamAccess.list);
            pManager.AddNumberParameter("Sun Search Radius", "SR", "IDW search radius for sampling the sun field.", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Radius", "R", "Base tube radius.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Radius Factor", "RF", "Radius multiplier at full sun (lerped by sun value).", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Blend", "B", "Base smooth-min fillet size at junctions.", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("Blend Factor", "BF", "Blend multiplier at full sun.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Voxel Size", "V", "SDF grid cell size. Smaller = finer + slower.", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("Noise Amp", "NA", "Amplitude of radius noise wobble (0 = off).", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Noise Scale", "NS", "Spatial scale of the radius noise.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Taper", "T", "End-taper amount along each curve (0 = off).", GH_ParamAccess.item, 0.0);
            pManager.AddIntegerParameter("Smooth Passes", "S", "Laplacian smoothing passes on the final mesh.", GH_ParamAccess.item, 0);

            pManager[1].Optional = true; // Sun Points
            pManager[2].Optional = true; // Sun Values
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Resulting metaball mesh.", GH_ParamAccess.item);
            pManager.AddGenericParameter("Debug Points", "D", "Coloured sample point cloud (sun normalisation debug).", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var curves = new List<Curve>();
            var sunPoints = new List<Point3d>();
            var sunValues = new List<double>();
            double sunSearchRadius = 10.0, radius = 1.0, radiusFactor = 1.0;
            double blend = 0.5, blendFactor = 1.0, voxelSize = 0.5;
            double noiseAmp = 0.0, noiseScale = 1.0, taper = 0.0;
            int smoothPasses = 0;

            DA.GetDataList(0, curves);
            DA.GetDataList(1, sunPoints);
            DA.GetDataList(2, sunValues);
            DA.GetData(3, ref sunSearchRadius);
            DA.GetData(4, ref radius);
            DA.GetData(5, ref radiusFactor);
            DA.GetData(6, ref blend);
            DA.GetData(7, ref blendFactor);
            DA.GetData(8, ref voxelSize);
            DA.GetData(9, ref noiseAmp);
            DA.GetData(10, ref noiseScale);
            DA.GetData(11, ref taper);
            DA.GetData(12, ref smoothPasses);

            // Guard against a hang: a zero/negative voxel size makes the stamp
            // spans effectively infinite.
            if (voxelSize <= 1e-9)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Voxel Size must be greater than 0.");
                return;
            }

            // Guard the parallel-list assumption the script relied on.
            if (sunPoints.Count != sunValues.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    "Sun Points and Sun Values differ in length; sun modulation disabled.");
                sunPoints = new List<Point3d>();
                sunValues = new List<double>();
            }

            // build sun hash first
            BuildSunHash(sunPoints, sunValues, sunSearchRadius);

            // get sun normalisation range
            double sMin = double.MaxValue, sMax = double.MinValue;
            for (int i = 0; i < sunValues.Count; i++)
            {
                if (sunValues[i] < sMin) sMin = sunValues[i];
                if (sunValues[i] > sMax) sMax = sunValues[i];
            }
            double sRange = sMax - sMin;

            // max radius i will ever have- used for margin sizing
            double rPossible = radius * Math.Max(1.0, radiusFactor) * (1 + noiseAmp);
            double kPossible = blend * Math.Max(1.0, blendFactor);
            int spanMax = (int)Math.Ceiling((rPossible + kPossible * 2) / voxelSize);

            double sampleSpacing = voxelSize * 0.5;
            double rMin = radius * 0.05;

            // build sample list
            List<Sample> samples = new List<Sample>();
            Point3d origin = Point3d.Unset;

            PointCloud debugCloud = new PointCloud();

            foreach (Curve c in curves)
            {
                double len = c.GetLength();
                int n = Math.Max(8, (int)Math.Ceiling(len / sampleSpacing));

                Point3d[] sp;
                double[] ts = c.DivideByCount(n, true, out sp);

                Interval dom = c.Domain;
                double domLen = dom.Length;

                for (int i = 0; i < sp.Length; i++)
                {
                    Point3d p = sp[i];
                    double tnorm = (ts[i] - dom.Min) / domLen;

                    double r = radius;
                    double k = blend;

                    // sun modulation - lerp r,k toward *Factor versions based on sun
                    double sun = SampleSun(p, sunSearchRadius);
                    double sNorm;
                    if (sun < 0) sNorm = 0;
                    else sNorm = (sun - sMin) / sRange;
                    if (sNorm < 0) sNorm = 0;
                    if (sNorm > 1) sNorm = 1;

                    r *= 1.0 + sNorm * (radiusFactor - 1.0);
                    k *= 1.0 + sNorm * (blendFactor - 1.0);

                    // noise wobble on radius
                    if (noiseAmp > 0)
                    {
                        double n3 = Noise3D(p.X / noiseScale, p.Y / noiseScale, p.Z / noiseScale);
                        r *= 1.0 + noiseAmp * n3;
                    }

                    // taper at curve ends
                    if (taper > 0)
                    {
                        double bump = 1.0 - 4.0 * tnorm * (1.0 - tnorm);
                        r *= 1.0 - taper * bump * bump;
                    }

                    if (r < rMin) r = rMin;
                    if (k < 1e-6) k = 1e-6;

                    samples.Add(new Sample { p = p, r = (float)r, k = (float)k });

                    debugCloud.Add(p, ValueToColor(sNorm));

                    // track lower-corner of bounding region
                    if (origin == Point3d.Unset) origin = p;
                    else
                    {
                        if (p.X < origin.X) origin.X = p.X;
                        if (p.Y < origin.Y) origin.Y = p.Y;
                        if (p.Z < origin.Z) origin.Z = p.Z;
                    }
                }
            }
            if (samples.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No samples generated.");
                return;
            }

            DA.SetData(1, debugCloud);

            // back the origin off a few voxels so stamps near the corner don't
            // get clipped against the grid edge
            origin = new Point3d(
                origin.X - voxelSize * (spanMax + 2),
                origin.Y - voxelSize * (spanMax + 2),
                origin.Z - voxelSize * (spanMax + 2));

            // stamp SDF spheres into sparse grid via smin
            Dictionary<long, float> grid = new Dictionary<long, float>(samples.Count * 32);

            foreach (Sample s in samples)
            {
                double range = s.r + s.k * 2.0;
                double range2 = range * range;
                int span = (int)Math.Ceiling(range / voxelSize);

                int ci = (int)Math.Floor((s.p.X - origin.X) / voxelSize);
                int cj = (int)Math.Floor((s.p.Y - origin.Y) / voxelSize);
                int ck = (int)Math.Floor((s.p.Z - origin.Z) / voxelSize);

                // walk the bounding box, skip voxels outside influence
                for (int dk = -span; dk <= span; dk++)
                {
                    double z = origin.Z + (ck + dk) * voxelSize;
                    double dz = z - s.p.Z;
                    double dz2 = dz * dz;
                    if (dz2 > range2) continue;

                    for (int dj = -span; dj <= span; dj++)
                    {
                        double y = origin.Y + (cj + dj) * voxelSize;
                        double dy = y - s.p.Y;
                        double dyz2 = dy * dy + dz2;
                        if (dyz2 > range2) continue;

                        for (int di = -span; di <= span; di++)
                        {
                            double x = origin.X + (ci + di) * voxelSize;
                            double dx = x - s.p.X;
                            double d2 = dx * dx + dyz2;
                            if (d2 > range2) continue;

                            float sdf = (float)(Math.Sqrt(d2) - s.r);
                            long key = Pack(ci + di, cj + dj, ck + dk);

                            float existing;
                            if (grid.TryGetValue(key, out existing))
                                grid[key] = SMin(existing, sdf, s.k); // local blend
                            else
                                grid[key] = sdf;
                        }
                    }
                }
            }

            // collect cubes that touch any stamped voxel
            HashSet<long> activeCubes = new HashSet<long>();
            foreach (long key in grid.Keys)
            {
                int i, j, k;
                Unpack(key, out i, out j, out k);
                for (int dk = -1; dk <= 0; dk++)
                    for (int dj = -1; dj <= 0; dj++)
                        for (int di = -1; di <= 0; di++)
                            activeCubes.Add(Pack(i + di, j + dj, k + dk));
            }

            // marching cubes
            Mesh result = new Mesh();
            Dictionary<long, int> edgeCache = new Dictionary<long, int>(activeCubes.Count * 3);

            const float ISO = 0f;
            const float MISSING = 1e6f;   // stand-in for "voxel not stamped" = far outside

            foreach (long cubeKey in activeCubes)
            {
                int ci, cj, ck;
                Unpack(cubeKey, out ci, out cj, out ck);

                // grab the 8 corner SDF values
                float[] v = new float[8];
                for (int c = 0; c < 8; c++)
                {
                    int[] o = CornerOffsets[c];
                    long k = Pack(ci + o[0], cj + o[1], ck + o[2]);
                    float val;
                    v[c] = grid.TryGetValue(k, out val) ? val : MISSING;
                }

                // build the 8-bit cube index (which corners are inside)
                int cubeIdx = 0;
                if (v[0] < ISO) cubeIdx |= 1;
                if (v[1] < ISO) cubeIdx |= 2;
                if (v[2] < ISO) cubeIdx |= 4;
                if (v[3] < ISO) cubeIdx |= 8;
                if (v[4] < ISO) cubeIdx |= 16;
                if (v[5] < ISO) cubeIdx |= 32;
                if (v[6] < ISO) cubeIdx |= 64;
                if (v[7] < ISO) cubeIdx |= 128;

                int edges = EdgeTable[cubeIdx];
                if (edges == 0) continue; // wholly inside or outside

                // generate vertices for crossed edges only
                int[] vi = new int[12];
                if ((edges & 1) != 0) vi[0] = EdgeVert(result, edgeCache, ci, cj, ck, 0, v, ISO, origin, voxelSize);
                if ((edges & 2) != 0) vi[1] = EdgeVert(result, edgeCache, ci, cj, ck, 1, v, ISO, origin, voxelSize);
                if ((edges & 4) != 0) vi[2] = EdgeVert(result, edgeCache, ci, cj, ck, 2, v, ISO, origin, voxelSize);
                if ((edges & 8) != 0) vi[3] = EdgeVert(result, edgeCache, ci, cj, ck, 3, v, ISO, origin, voxelSize);
                if ((edges & 16) != 0) vi[4] = EdgeVert(result, edgeCache, ci, cj, ck, 4, v, ISO, origin, voxelSize);
                if ((edges & 32) != 0) vi[5] = EdgeVert(result, edgeCache, ci, cj, ck, 5, v, ISO, origin, voxelSize);
                if ((edges & 64) != 0) vi[6] = EdgeVert(result, edgeCache, ci, cj, ck, 6, v, ISO, origin, voxelSize);
                if ((edges & 128) != 0) vi[7] = EdgeVert(result, edgeCache, ci, cj, ck, 7, v, ISO, origin, voxelSize);
                if ((edges & 256) != 0) vi[8] = EdgeVert(result, edgeCache, ci, cj, ck, 8, v, ISO, origin, voxelSize);
                if ((edges & 512) != 0) vi[9] = EdgeVert(result, edgeCache, ci, cj, ck, 9, v, ISO, origin, voxelSize);
                if ((edges & 1024) != 0) vi[10] = EdgeVert(result, edgeCache, ci, cj, ck, 10, v, ISO, origin, voxelSize);
                if ((edges & 2048) != 0) vi[11] = EdgeVert(result, edgeCache, ci, cj, ck, 11, v, ISO, origin, voxelSize);

                // emit triangles
                for (int t = 0; TriTable[cubeIdx, t] != -1; t += 3)
                    result.Faces.AddFace(
                        vi[TriTable[cubeIdx, t]],
                        vi[TriTable[cubeIdx, t + 1]],
                        vi[TriTable[cubeIdx, t + 2]]);
            }

            // cleanup
            result.Vertices.CombineIdentical(true, true);
            result.Faces.CullDegenerateFaces();
            result.UnifyNormals();

            for (int s = 0; s < smoothPasses; s++)
                result.Smooth(0.5, true, true, true, true, SmoothingCoordinateSystem.World);

            result.Normals.ComputeNormals();
            DA.SetData(0, result);
        }

        // ===================== helpers (unchanged from the script) =====================

        // 21-bit packing - fits 3 axes into one long
        const int PACK_OFFSET = 1 << 20;
        const long PACK_MASK = (1L << 21) - 1L;

        long Pack(int i, int j, int k)
        {
            long li = (long)(i + PACK_OFFSET);
            long lj = (long)(j + PACK_OFFSET);
            long lk = (long)(k + PACK_OFFSET);
            return (li << 42) | (lj << 21) | lk;
        }
        void Unpack(long key, out int i, out int j, out int k)
        {
            long lk = key & PACK_MASK;
            long lj = (key >> 21) & PACK_MASK;
            long li = (key >> 42) & PACK_MASK;
            i = (int)li - PACK_OFFSET;
            j = (int)lj - PACK_OFFSET;
            k = (int)lk - PACK_OFFSET;
        }

        // cheap 3d noise from sine waves
        double Noise3D(double x, double y, double z)
        {
            double n = Math.Sin(x * 1.7 + y * 2.3 + z * 1.1)
                    + 0.5 * Math.Sin(x * 3.1 - y * 1.7 + z * 2.9)
                    + 0.25 * Math.Sin(x * 5.3 + y * 4.7 - z * 6.1);
            return n * 0.5714;
        }

        // smooth-min, k = fillet size at joint
        float SMin(float a, float b, float k)
        {
            if (k <= 1e-6f) return Math.Min(a, b);
            float h = Math.Max(k - Math.Abs(a - b), 0f) / k;
            return Math.Min(a, b) - h * h * h * k * (1f / 6f);
        }

        // per-sample stamp
        struct Sample
        {
            public Point3d p;
            public float r;
            public float k;  // local blend
        }

        // sun spatial hash
        Dictionary<long, List<int>> sunHash;
        double sunCell;
        List<Point3d> sunPts;
        List<double> sunVals;

        long SunKey(int i, int j, int k)
        {
            return ((long)i) * 73856093L ^ ((long)j) * 19349663L ^ ((long)k) * 83492791L;
        }

        void BuildSunHash(List<Point3d> pts, List<double> vals, double cell)
        {
            sunPts = pts;
            sunVals = vals;
            sunCell = cell <= 1e-9 ? 1.0 : cell; // guard divide-by-zero
            sunHash = new Dictionary<long, List<int>>();

            for (int i = 0; i < pts.Count; i++)
            {
                Point3d p = pts[i];
                int ix = (int)Math.Floor(p.X / sunCell);
                int iy = (int)Math.Floor(p.Y / sunCell);
                int iz = (int)Math.Floor(p.Z / sunCell);
                long key = SunKey(ix, iy, iz);
                List<int> bucket;
                if (!sunHash.TryGetValue(key, out bucket)) { bucket = new List<int>(); sunHash[key] = bucket; }
                bucket.Add(i);
            }
        }

        // IDW lookup. Returns -1 when nothing in range -> caller treats as shaded
        double SampleSun(Point3d p, double radius)
        {
            double r2 = radius * radius;
            int ix = (int)Math.Floor(p.X / sunCell);
            int iy = (int)Math.Floor(p.Y / sunCell);
            int iz = (int)Math.Floor(p.Z / sunCell);

            double weightSum = 0;
            double valueSum = 0;
            bool found = false;

            // 27-cell scan around query
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        List<int> bucket;
                        if (!sunHash.TryGetValue(SunKey(ix + dx, iy + dy, iz + dz), out bucket)) continue;
                        foreach (int idx in bucket)
                        {
                            double d2 = p.DistanceToSquared(sunPts[idx]);
                            if (d2 > r2) continue;
                            double w = 1.0 / (Math.Sqrt(d2) + 1e-6);
                            valueSum += sunVals[idx] * w;
                            weightSum += w;
                            found = true;
                        }
                    }

            if (!found) return -1;
            return valueSum / weightSum;
        }

        // edge interp + caching, linear interp on corner SDFs
        int EdgeVert(Mesh m, Dictionary<long, int> cache,
                     int ci, int cj, int ck, int edge, float[] v, float iso,
                     Point3d origin, double s)
        {
            int[] e = EdgeCorners[edge];
            int[] o0 = CornerOffsets[e[0]];
            int[] o1 = CornerOffsets[e[1]];

            int gi0 = ci + o0[0], gj0 = cj + o0[1], gk0 = ck + o0[2];
            int gi1 = ci + o1[0], gj1 = cj + o1[1], gk1 = ck + o1[2];

            long a = Pack(gi0, gj0, gk0);
            long b = Pack(gi1, gj1, gk1);
            // canonical key for the (unordered) edge - so neighbouring cubes share the same vertex
            long key = unchecked(a < b ? (a * 1000003L + b) : (b * 1000003L + a));

            int existing;
            if (cache.TryGetValue(key, out existing)) return existing;

            float v0 = v[e[0]];
            float v1 = v[e[1]];
            float t = (Math.Abs(v1 - v0) < 1e-9f) ? 0.5f : (iso - v0) / (v1 - v0);
            if (t < 0) t = 0;
            if (t > 1) t = 1;

            Point3d p0 = new Point3d(origin.X + gi0 * s, origin.Y + gj0 * s, origin.Z + gk0 * s);
            Point3d p1 = new Point3d(origin.X + gi1 * s, origin.Y + gj1 * s, origin.Z + gk1 * s);
            Point3d p = p0 + (p1 - p0) * t;

            int idx = m.Vertices.Add(p);
            cache[key] = idx;
            return idx;
        }

        // blue -> red ramp for debug
        Color ValueToColor(double t)
        {
            if (t < 0) t = 0; if (t > 1) t = 1;
            double hue = (1.0 - t) * 240.0;
            return ColorFromHSV(hue, 1.0, 1.0);
        }

        static Color ColorFromHSV(double h, double s, double v)
        {
            h = ((h % 360) + 360) % 360; // normalise to 360
            int hi = (int)Math.Floor(h / 60) % 6;
            double f = h / 60 - Math.Floor(h / 60);

            double p = v * (1 - s);
            double q = v * (1 - f * s);
            double t = v * (1 - (1 - f) * s);

            double[][] sectors = new double[][]
            {
                new[] {v,t,p}, // hi = 0 to 5
                new[] {q,v,p},
                new[] {p,v,t},
                new[] {p,q,v},
                new[] {t,p,v},
                new[] {v,p,q}
            };

            double[] rgb = sectors[hi];

            return Color.FromArgb(255,
                (int)Math.Round(rgb[0] * 255),
                (int)Math.Round(rgb[1] * 255),
                (int)Math.Round(rgb[2] * 255));
        }

        // cube topology
        static readonly int[][] CornerOffsets = new int[][] {
            new int[]{0,0,0}, new int[]{1,0,0}, new int[]{1,1,0}, new int[]{0,1,0},
            new int[]{0,0,1}, new int[]{1,0,1}, new int[]{1,1,1}, new int[]{0,1,1}
            };
        static readonly int[][] EdgeCorners = new int[][] {
            new int[]{0,1}, new int[]{1,2}, new int[]{2,3}, new int[]{3,0},
            new int[]{4,5}, new int[]{5,6}, new int[]{6,7}, new int[]{7,4},
            new int[]{0,4}, new int[]{1,5}, new int[]{2,6}, new int[]{3,7}
            };

        // marching cubes lookup tables
        static readonly int[] EdgeTable = new int[] {
            0x0  ,0x109,0x203,0x30a,0x406,0x50f,0x605,0x70c,0x80c,0x905,0xa0f,0xb06,0xc0a,0xd03,0xe09,0xf00,
            0x190,0x99 ,0x393,0x29a,0x596,0x49f,0x795,0x69c,0x99c,0x895,0xb9f,0xa96,0xd9a,0xc93,0xf99,0xe90,
            0x230,0x339,0x33 ,0x13a,0x636,0x73f,0x435,0x53c,0xa3c,0xb35,0x83f,0x936,0xe3a,0xf33,0xc39,0xd30,
            0x3a0,0x2a9,0x1a3,0xaa ,0x7a6,0x6af,0x5a5,0x4ac,0xbac,0xaa5,0x9af,0x8a6,0xfaa,0xea3,0xda9,0xca0,
            0x460,0x569,0x663,0x76a,0x66 ,0x16f,0x265,0x36c,0xc6c,0xd65,0xe6f,0xf66,0x86a,0x963,0xa69,0xb60,
            0x5f0,0x4f9,0x7f3,0x6fa,0x1f6,0xff ,0x3f5,0x2fc,0xdfc,0xcf5,0xfff,0xef6,0x9fa,0x8f3,0xbf9,0xaf0,
            0x650,0x759,0x453,0x55a,0x256,0x35f,0x55 ,0x15c,0xe5c,0xf55,0xc5f,0xd56,0xa5a,0xb53,0x859,0x950,
            0x7c0,0x6c9,0x5c3,0x4ca,0x3c6,0x2cf,0x1c5,0xcc ,0xfcc,0xec5,0xdcf,0xcc6,0xbca,0xac3,0x9c9,0x8c0,
            0x8c0,0x9c9,0xac3,0xbca,0xcc6,0xdcf,0xec5,0xfcc,0xcc ,0x1c5,0x2cf,0x3c6,0x4ca,0x5c3,0x6c9,0x7c0,
            0x950,0x859,0xb53,0xa5a,0xd56,0xc5f,0xf55,0xe5c,0x15c,0x55 ,0x35f,0x256,0x55a,0x453,0x759,0x650,
            0xaf0,0xbf9,0x8f3,0x9fa,0xef6,0xfff,0xcf5,0xdfc,0x2fc,0x3f5,0xff ,0x1f6,0x6fa,0x7f3,0x4f9,0x5f0,
            0xb60,0xa69,0x963,0x86a,0xf66,0xe6f,0xd65,0xc6c,0x36c,0x265,0x16f,0x66 ,0x76a,0x663,0x569,0x460,
            0xca0,0xda9,0xea3,0xfaa,0x8a6,0x9af,0xaa5,0xbac,0x4ac,0x5a5,0x6af,0x7a6,0xaa ,0x1a3,0x2a9,0x3a0,
            0xd30,0xc39,0xf33,0xe3a,0x936,0x83f,0xb35,0xa3c,0x53c,0x435,0x73f,0x636,0x13a,0x33 ,0x339,0x230,
            0xe90,0xf99,0xc93,0xd9a,0xa96,0xb9f,0x895,0x99c,0x69c,0x795,0x49f,0x596,0x29a,0x393,0x99 ,0x190,
            0xf00,0xe09,0xd03,0xc0a,0xb06,0xa0f,0x905,0x80c,0x70c,0x605,0x50f,0x406,0x30a,0x203,0x109,0x0
            };

        static readonly int[,] TriTable = new int[256, 16] {
            {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,8,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {0,1,9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{1,8,3,9,8,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {1,2,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,8,3,1,2,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {9,2,10,0,2,9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{2,8,3,2,10,8,10,9,8,-1,-1,-1,-1,-1,-1,-1},
            {3,11,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,11,2,8,11,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {1,9,0,2,3,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{1,11,2,1,9,11,9,8,11,-1,-1,-1,-1,-1,-1,-1},
            {3,10,1,11,10,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,10,1,0,8,10,8,11,10,-1,-1,-1,-1,-1,-1,-1},
            {3,9,0,3,11,9,11,10,9,-1,-1,-1,-1,-1,-1,-1},{9,8,10,10,8,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {4,7,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{4,3,0,7,3,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {0,1,9,8,4,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{4,1,9,4,7,1,7,3,1,-1,-1,-1,-1,-1,-1,-1},
            {1,2,10,8,4,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{3,4,7,3,0,4,1,2,10,-1,-1,-1,-1,-1,-1,-1},
            {9,2,10,9,0,2,8,4,7,-1,-1,-1,-1,-1,-1,-1},{2,10,9,2,9,7,2,7,3,7,9,4,-1,-1,-1,-1},
            {8,4,7,3,11,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{11,4,7,11,2,4,2,0,4,-1,-1,-1,-1,-1,-1,-1},
            {9,0,1,8,4,7,2,3,11,-1,-1,-1,-1,-1,-1,-1},{4,7,11,9,4,11,9,11,2,9,2,1,-1,-1,-1,-1},
            {3,10,1,3,11,10,7,8,4,-1,-1,-1,-1,-1,-1,-1},{1,11,10,1,4,11,1,0,4,7,11,4,-1,-1,-1,-1},
            {4,7,8,9,0,11,9,11,10,11,0,3,-1,-1,-1,-1},{4,7,11,4,11,9,9,11,10,-1,-1,-1,-1,-1,-1,-1},
            {9,5,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{9,5,4,0,8,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {0,5,4,1,5,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{8,5,4,8,3,5,3,1,5,-1,-1,-1,-1,-1,-1,-1},
            {1,2,10,9,5,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{3,0,8,1,2,10,4,9,5,-1,-1,-1,-1,-1,-1,-1},
            {5,2,10,5,4,2,4,0,2,-1,-1,-1,-1,-1,-1,-1},{2,10,5,3,2,5,3,5,4,3,4,8,-1,-1,-1,-1},
            {9,5,4,2,3,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,11,2,0,8,11,4,9,5,-1,-1,-1,-1,-1,-1,-1},
            {0,5,4,0,1,5,2,3,11,-1,-1,-1,-1,-1,-1,-1},{2,1,5,2,5,8,2,8,11,4,8,5,-1,-1,-1,-1},
            {10,3,11,10,1,3,9,5,4,-1,-1,-1,-1,-1,-1,-1},{4,9,5,0,8,1,8,10,1,8,11,10,-1,-1,-1,-1},
            {5,4,0,5,0,11,5,11,10,11,0,3,-1,-1,-1,-1},{5,4,8,5,8,10,10,8,11,-1,-1,-1,-1,-1,-1,-1},
            {9,7,8,5,7,9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{9,3,0,9,5,3,5,7,3,-1,-1,-1,-1,-1,-1,-1},
            {0,7,8,0,1,7,1,5,7,-1,-1,-1,-1,-1,-1,-1},{1,5,3,3,5,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {9,7,8,9,5,7,10,1,2,-1,-1,-1,-1,-1,-1,-1},{10,1,2,9,5,0,5,3,0,5,7,3,-1,-1,-1,-1},
            {8,0,2,8,2,5,8,5,7,10,5,2,-1,-1,-1,-1},{2,10,5,2,5,3,3,5,7,-1,-1,-1,-1,-1,-1,-1},
            {7,9,5,7,8,9,3,11,2,-1,-1,-1,-1,-1,-1,-1},{9,5,7,9,7,2,9,2,0,2,7,11,-1,-1,-1,-1},
            {2,3,11,0,1,8,1,7,8,1,5,7,-1,-1,-1,-1},{11,2,1,11,1,7,7,1,5,-1,-1,-1,-1,-1,-1,-1},
            {9,5,8,8,5,7,10,1,3,10,3,11,-1,-1,-1,-1},{5,7,0,5,0,9,7,11,0,1,0,10,11,10,0,-1},
            {11,10,0,11,0,3,10,5,0,8,0,7,5,7,0,-1},{11,10,5,7,11,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {10,6,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,8,3,5,10,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {9,0,1,5,10,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{1,8,3,1,9,8,5,10,6,-1,-1,-1,-1,-1,-1,-1},
            {1,6,5,2,6,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{1,6,5,1,2,6,3,0,8,-1,-1,-1,-1,-1,-1,-1},
            {9,6,5,9,0,6,0,2,6,-1,-1,-1,-1,-1,-1,-1},{5,9,8,5,8,2,5,2,6,3,2,8,-1,-1,-1,-1},
            {2,3,11,10,6,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{11,0,8,11,2,0,10,6,5,-1,-1,-1,-1,-1,-1,-1},
            {0,1,9,2,3,11,5,10,6,-1,-1,-1,-1,-1,-1,-1},{5,10,6,1,9,2,9,11,2,9,8,11,-1,-1,-1,-1},
            {6,3,11,6,5,3,5,1,3,-1,-1,-1,-1,-1,-1,-1},{0,8,11,0,11,5,0,5,1,5,11,6,-1,-1,-1,-1},
            {3,11,6,0,3,6,0,6,5,0,5,9,-1,-1,-1,-1},{6,5,9,6,9,11,11,9,8,-1,-1,-1,-1,-1,-1,-1},
            {5,10,6,4,7,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{4,3,0,4,7,3,6,5,10,-1,-1,-1,-1,-1,-1,-1},
            {1,9,0,5,10,6,8,4,7,-1,-1,-1,-1,-1,-1,-1},{10,6,5,1,9,7,1,7,3,7,9,4,-1,-1,-1,-1},
            {6,1,2,6,5,1,4,7,8,-1,-1,-1,-1,-1,-1,-1},{1,2,5,5,2,6,3,0,4,3,4,7,-1,-1,-1,-1},
            {8,4,7,9,0,5,0,6,5,0,2,6,-1,-1,-1,-1},{7,3,9,7,9,4,3,2,9,5,9,6,2,6,9,-1},
            {3,11,2,7,8,4,10,6,5,-1,-1,-1,-1,-1,-1,-1},{5,10,6,4,7,2,4,2,0,2,7,11,-1,-1,-1,-1},
            {0,1,9,4,7,8,2,3,11,5,10,6,-1,-1,-1,-1},{9,2,1,9,11,2,9,4,11,7,11,4,5,10,6,-1},
            {8,4,7,3,11,5,3,5,1,5,11,6,-1,-1,-1,-1},{5,1,11,5,11,6,1,0,11,7,11,4,0,4,11,-1},
            {0,5,9,0,6,5,0,3,6,11,6,3,8,4,7,-1},{6,5,9,6,9,11,4,7,9,7,11,9,-1,-1,-1,-1},
            {10,4,9,6,4,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{4,10,6,4,9,10,0,8,3,-1,-1,-1,-1,-1,-1,-1},
            {10,0,1,10,6,0,6,4,0,-1,-1,-1,-1,-1,-1,-1},{8,3,1,8,1,6,8,6,4,6,1,10,-1,-1,-1,-1},
            {1,4,9,1,2,4,2,6,4,-1,-1,-1,-1,-1,-1,-1},{3,0,8,1,2,9,2,4,9,2,6,4,-1,-1,-1,-1},
            {0,2,4,4,2,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{8,3,2,8,2,4,4,2,6,-1,-1,-1,-1,-1,-1,-1},
            {10,4,9,10,6,4,11,2,3,-1,-1,-1,-1,-1,-1,-1},{0,8,2,2,8,11,4,9,10,4,10,6,-1,-1,-1,-1},
            {3,11,2,0,1,6,0,6,4,6,1,10,-1,-1,-1,-1},{6,4,1,6,1,10,4,8,1,2,1,11,8,11,1,-1},
            {9,6,4,9,3,6,9,1,3,11,6,3,-1,-1,-1,-1},{8,11,1,8,1,0,11,6,1,9,1,4,6,4,1,-1},
            {3,11,6,3,6,0,0,6,4,-1,-1,-1,-1,-1,-1,-1},{6,4,8,11,6,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {7,10,6,7,8,10,8,9,10,-1,-1,-1,-1,-1,-1,-1},{0,7,3,0,10,7,0,9,10,6,7,10,-1,-1,-1,-1},
            {10,6,7,1,10,7,1,7,8,1,8,0,-1,-1,-1,-1},{10,6,7,10,7,1,1,7,3,-1,-1,-1,-1,-1,-1,-1},
            {1,2,6,1,6,8,1,8,9,8,6,7,-1,-1,-1,-1},{2,6,9,2,9,1,6,7,9,0,9,3,7,3,9,-1},
            {7,8,0,7,0,6,6,0,2,-1,-1,-1,-1,-1,-1,-1},{7,3,2,6,7,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {2,3,11,10,6,8,10,8,9,8,6,7,-1,-1,-1,-1},{2,0,7,2,7,11,0,9,7,6,7,10,9,10,7,-1},
            {1,8,0,1,7,8,1,10,7,6,7,10,2,3,11,-1},{11,2,1,11,1,7,10,6,1,6,7,1,-1,-1,-1,-1},
            {8,9,6,8,6,7,9,1,6,11,6,3,1,3,6,-1},{0,9,1,11,6,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {7,8,0,7,0,6,3,11,0,11,6,0,-1,-1,-1,-1},{7,11,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {7,6,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{3,0,8,11,7,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {0,1,9,11,7,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{8,1,9,8,3,1,11,7,6,-1,-1,-1,-1,-1,-1,-1},
            {10,1,2,6,11,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{1,2,10,3,0,8,6,11,7,-1,-1,-1,-1,-1,-1,-1},
            {2,9,0,2,10,9,6,11,7,-1,-1,-1,-1,-1,-1,-1},{6,11,7,2,10,3,10,8,3,10,9,8,-1,-1,-1,-1},
            {7,2,3,6,2,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{7,0,8,7,6,0,6,2,0,-1,-1,-1,-1,-1,-1,-1},
            {2,7,6,2,3,7,0,1,9,-1,-1,-1,-1,-1,-1,-1},{1,6,2,1,8,6,1,9,8,8,7,6,-1,-1,-1,-1},
            {10,7,6,10,1,7,1,3,7,-1,-1,-1,-1,-1,-1,-1},{10,7,6,1,7,10,1,8,7,1,0,8,-1,-1,-1,-1},
            {0,3,7,0,7,10,0,10,9,6,10,7,-1,-1,-1,-1},{7,6,10,7,10,8,8,10,9,-1,-1,-1,-1,-1,-1,-1},
            {6,8,4,11,8,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{3,6,11,3,0,6,0,4,6,-1,-1,-1,-1,-1,-1,-1},
            {8,6,11,8,4,6,9,0,1,-1,-1,-1,-1,-1,-1,-1},{9,4,6,9,6,3,9,3,1,11,3,6,-1,-1,-1,-1},
            {6,8,4,6,11,8,2,10,1,-1,-1,-1,-1,-1,-1,-1},{1,2,10,3,0,11,0,6,11,0,4,6,-1,-1,-1,-1},
            {4,11,8,4,6,11,0,2,9,2,10,9,-1,-1,-1,-1},{10,9,3,10,3,2,9,4,3,11,3,6,4,6,3,-1},
            {8,2,3,8,4,2,4,6,2,-1,-1,-1,-1,-1,-1,-1},{0,4,2,4,6,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {1,9,0,2,3,4,2,4,6,4,3,8,-1,-1,-1,-1},{1,9,4,1,4,2,2,4,6,-1,-1,-1,-1,-1,-1,-1},
            {8,1,3,8,6,1,8,4,6,6,10,1,-1,-1,-1,-1},{10,1,0,10,0,6,6,0,4,-1,-1,-1,-1,-1,-1,-1},
            {4,6,3,4,3,8,6,10,3,0,3,9,10,9,3,-1},{10,9,4,6,10,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {4,9,5,7,6,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,8,3,4,9,5,11,7,6,-1,-1,-1,-1,-1,-1,-1},
            {5,0,1,5,4,0,7,6,11,-1,-1,-1,-1,-1,-1,-1},{11,7,6,8,3,4,3,5,4,3,1,5,-1,-1,-1,-1},
            {9,5,4,10,1,2,7,6,11,-1,-1,-1,-1,-1,-1,-1},{6,11,7,1,2,10,0,8,3,4,9,5,-1,-1,-1,-1},
            {7,6,11,5,4,10,4,2,10,4,0,2,-1,-1,-1,-1},{3,4,8,3,5,4,3,2,5,10,5,2,11,7,6,-1},
            {7,2,3,7,6,2,5,4,9,-1,-1,-1,-1,-1,-1,-1},{9,5,4,0,8,6,0,6,2,6,8,7,-1,-1,-1,-1},
            {3,6,2,3,7,6,1,5,0,5,4,0,-1,-1,-1,-1},{6,2,8,6,8,7,2,1,8,4,8,5,1,5,8,-1},
            {9,5,4,10,1,6,1,7,6,1,3,7,-1,-1,-1,-1},{1,6,10,1,7,6,1,0,7,8,7,0,9,5,4,-1},
            {4,0,10,4,10,5,0,3,10,6,10,7,3,7,10,-1},{7,6,10,7,10,8,5,4,10,4,8,10,-1,-1,-1,-1},
            {6,9,5,6,11,9,11,8,9,-1,-1,-1,-1,-1,-1,-1},{3,6,11,0,6,3,0,5,6,0,9,5,-1,-1,-1,-1},
            {0,11,8,0,5,11,0,1,5,5,6,11,-1,-1,-1,-1},{6,11,3,6,3,5,5,3,1,-1,-1,-1,-1,-1,-1,-1},
            {1,2,10,9,5,11,9,11,8,11,5,6,-1,-1,-1,-1},{0,11,3,0,6,11,0,9,6,5,6,9,1,2,10,-1},
            {11,8,5,11,5,6,8,0,5,10,5,2,0,2,5,-1},{6,11,3,6,3,5,2,10,3,10,5,3,-1,-1,-1,-1},
            {5,8,9,5,2,8,5,6,2,3,8,2,-1,-1,-1,-1},{9,5,6,9,6,0,0,6,2,-1,-1,-1,-1,-1,-1,-1},
            {1,5,8,1,8,0,5,6,8,3,8,2,6,2,8,-1},{1,5,6,2,1,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {1,3,6,1,6,10,3,8,6,5,6,9,8,9,6,-1},{10,1,0,10,0,6,9,5,0,5,6,0,-1,-1,-1,-1},
            {0,3,8,5,6,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{10,5,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {11,5,10,7,5,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{11,5,10,11,7,5,8,3,0,-1,-1,-1,-1,-1,-1,-1},
            {5,11,7,5,10,11,1,9,0,-1,-1,-1,-1,-1,-1,-1},{10,7,5,10,11,7,9,8,1,8,3,1,-1,-1,-1,-1},
            {11,1,2,11,7,1,7,5,1,-1,-1,-1,-1,-1,-1,-1},{0,8,3,1,2,7,1,7,5,7,2,11,-1,-1,-1,-1},
            {9,7,5,9,2,7,9,0,2,2,11,7,-1,-1,-1,-1},{7,5,2,7,2,11,5,9,2,3,2,8,9,8,2,-1},
            {2,5,10,2,3,5,3,7,5,-1,-1,-1,-1,-1,-1,-1},{8,2,0,8,5,2,8,7,5,10,2,5,-1,-1,-1,-1},
            {9,0,1,5,10,3,5,3,7,3,10,2,-1,-1,-1,-1},{9,8,2,9,2,1,8,7,2,10,2,5,7,5,2,-1},
            {1,3,5,3,7,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,8,7,0,7,1,1,7,5,-1,-1,-1,-1,-1,-1,-1},
            {9,0,3,9,3,5,5,3,7,-1,-1,-1,-1,-1,-1,-1},{9,8,7,5,9,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {5,8,4,5,10,8,10,11,8,-1,-1,-1,-1,-1,-1,-1},{5,0,4,5,11,0,5,10,11,11,3,0,-1,-1,-1,-1},
            {0,1,9,8,4,10,8,10,11,10,4,5,-1,-1,-1,-1},{10,11,4,10,4,5,11,3,4,9,4,1,3,1,4,-1},
            {2,5,1,2,8,5,2,11,8,4,5,8,-1,-1,-1,-1},{0,4,11,0,11,3,4,5,11,2,11,1,5,1,11,-1},
            {0,2,5,0,5,9,2,11,5,4,5,8,11,8,5,-1},{9,4,5,2,11,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {2,5,10,3,5,2,3,4,5,3,8,4,-1,-1,-1,-1},{5,10,2,5,2,4,4,2,0,-1,-1,-1,-1,-1,-1,-1},
            {3,10,2,3,5,10,3,8,5,4,5,8,0,1,9,-1},{5,10,2,5,2,4,1,9,2,9,4,2,-1,-1,-1,-1},
            {8,4,5,8,5,3,3,5,1,-1,-1,-1,-1,-1,-1,-1},{0,4,5,1,0,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {8,4,5,8,5,3,9,0,5,0,3,5,-1,-1,-1,-1},{9,4,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {4,11,7,4,9,11,9,10,11,-1,-1,-1,-1,-1,-1,-1},{0,8,3,4,9,7,9,11,7,9,10,11,-1,-1,-1,-1},
            {1,10,11,1,11,4,1,4,0,7,4,11,-1,-1,-1,-1},{3,1,4,3,4,8,1,10,4,7,4,11,10,11,4,-1},
            {4,11,7,9,11,4,9,2,11,9,1,2,-1,-1,-1,-1},{9,7,4,9,11,7,9,1,11,2,11,1,0,8,3,-1},
            {11,7,4,11,4,2,2,4,0,-1,-1,-1,-1,-1,-1,-1},{11,7,4,11,4,2,8,3,4,3,2,4,-1,-1,-1,-1},
            {2,9,10,2,7,9,2,3,7,7,4,9,-1,-1,-1,-1},{9,10,7,9,7,4,10,2,7,8,7,0,2,0,7,-1},
            {3,7,10,3,10,2,7,4,10,1,10,0,4,0,10,-1},{1,10,2,8,7,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {4,9,1,4,1,7,7,1,3,-1,-1,-1,-1,-1,-1,-1},{4,9,1,4,1,7,0,8,1,8,7,1,-1,-1,-1,-1},
            {4,0,3,7,4,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{4,8,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {9,10,8,10,11,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{3,0,9,3,9,11,11,9,10,-1,-1,-1,-1,-1,-1,-1},
            {0,1,10,0,10,8,8,10,11,-1,-1,-1,-1,-1,-1,-1},{3,1,10,11,3,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {1,2,11,1,11,9,9,11,8,-1,-1,-1,-1,-1,-1,-1},{3,0,9,3,9,11,1,2,9,2,11,9,-1,-1,-1,-1},
            {0,2,11,8,0,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{3,2,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {2,3,8,2,8,10,10,8,9,-1,-1,-1,-1,-1,-1,-1},{9,10,2,0,9,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {2,3,8,2,8,10,0,1,8,1,10,8,-1,-1,-1,-1},{1,10,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {1,3,8,9,1,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{0,9,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
            {0,3,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
            };

        protected override Bitmap Icon =>
            new Bitmap(GetType().Assembly.GetManifestResourceStream("MyPlugin.Icons.CurveMetaballer.png"));

        public override GH_Exposure Exposure => GH_Exposure.primary;

        // Generate your own — unique per component, permanent once shipped.
        public override Guid ComponentGuid => new Guid("f2a5b8c3-9d41-4e62-8b7a-4c5d6e7f8a91");
    }
}
