using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using System.Linq;

public class Script_Instance : GH_ScriptInstance
{
    bool init = false;
    public static Random rnd;
    public Flock birds;
    public static int trailLength = 0;
    public static bool cutTrails = true;
    public static Brep boundingVolume = null;
    public static List<Brep> obstacleVolumes = new List<Brep>();
    public static List<Curve> teleportCurvesA = new List<Curve>();
    public static List<Curve> teleportCurvesB = new List<Curve>();

    public const double teleportRadius = 0.1; //hardcoded, if teleporting too early make smaller

    public static CFDField cfdField = null;
    public static double   windFactor = 0.0;

    //attractor detractor field
    public static List<Point3d> attractorPts  = new List<Point3d>();
    public static List<double>  attractorVals = new List<double>();
    public static double        attractorGain = 1.0;

    private void RunScript(
		bool reset,
		bool wrap,
		bool cutTrails,
		int num,
		double speed,
		int trailLen,
		double niegh,
		double coh,
		double alig,
		double sep,
		double sepDist,
		List<Point3d> attractorPoints,
		List<double> attractorValues,
		double attractorWeight,
		Brep boundingBox,
		Brep boidG,
		bool showBoids,
		List<Brep> obstacles,
		List<Curve> teleportA,
		List<Curve> teleportB,
		List<Point3d> cfdPoints,
		List<Vector3d> cfdVectors,
		Vector3d cfdMoveVec,
		double cfdRotAngle,
		double windF,
		ref object boidsTrails,
		ref object boidsLocations,
		ref object BoidDirections,
		ref object BoidsGeometry,
		ref object cfdCloud)
    {



        if(boundingBox == null) return;

        Script_Instance.trailLength = trailLen;
        Script_Instance.cutTrails = cutTrails;
        Script_Instance.boundingVolume = boundingBox;
        Script_Instance.obstacleVolumes = obstacles ?? new List<Brep>(); //incasee unconnected script has to calc
        Script_Instance.teleportCurvesA = teleportA ?? new List<Curve>();
        Script_Instance.teleportCurvesB = teleportB ?? new List<Curve>();
        Script_Instance.windFactor = windF;


        Script_Instance.attractorPts = attractorPoints ?? new List<Point3d>();
        Script_Instance.attractorVals = attractorValues ?? new List<double>();
        Script_Instance.attractorGain = attractorWeight;

        BoundingBox bb = boundingBox.GetBoundingBox(true);


        Transform cfdXform = Transform.Identity;
        if(Math.Abs(cfdRotAngle) > 1e-12)
        {
            double rad = cfdRotAngle * Math.PI / 180.0;
            cfdXform = Transform.Rotation(rad, Vector3d.ZAxis, Point3d.Origin);
        }
        if(cfdMoveVec.Length > 1e-12)
        cfdXform = Transform.Translation(cfdMoveVec) * cfdXform;

        if(init || reset || birds == null)
        {
            rnd = new Random();
            Point3d[] rndStartPoints = new Point3d[num];
            for (int i = 0; i < num; i++)
            {
                Point3d candidate;
                //int attempts = 0;
                //do 
                {
                candidate = new Point3d
                    (
                    bb.Min.X + rnd.NextDouble() * (bb.Max.X - bb.Min.X),
                    bb.Min.Y + rnd.NextDouble() * (bb.Max.Y - bb.Min.Y),
                    bb.Min.Z + rnd.NextDouble() * (bb.Max.Z - bb.Min.Z)
                    );

                //attempts++;
                } 
                
                //while (!boundingBox.IsPointInside(candidate, RhinoMath.SqrtEpsilon, false) && attempts < 1000);
                rndStartPoints[i] = candidate;
            }
        birds = new Flock(num, rndStartPoints);
        init = false;

//cfd on reset
        if (cfdPoints != null && cfdVectors != null
            && cfdPoints.Count > 0
            && cfdPoints.Count == cfdVectors.Count)
            {
                Script_Instance.cfdField = new CFDField(cfdPoints, cfdVectors, cfdXform);
            }

            else
            {
                Script_Instance.cfdField = null;
            }

        }

        else
        {
//pushes transforms without having to full reset
            if (Script_Instance.cfdField != null)
                Script_Instance.cfdField.UpdateTransform(cfdXform);
        }

        // Update flock
        birds.UpdateFlock(speed,alig, coh, sep, sepDist, niegh);
        birds.stayInBoundery(wrap);
        birds.checkTeleport();

        // Calculate outputs
        List<Point3d> boidsLocationsList = new List<Point3d>();
        List<Curve> boidsTrailsList = new List<Curve>();
        List<Vector3d> boidDirectionsList = new List<Vector3d>();
        List<Brep> boidsGeoList = new List<Brep>();

        for (int i=0; i < birds.birdsList.Count; i++)
        {
            boidsLocationsList.Add(birds.birdsList[i].location);
            boidDirectionsList.Add(birds.birdsList[i].direction);

            Brep geo = showBoids ? birds.birdsList[i].drawGeo(boidG) : null;
            if (geo != null) boidsGeoList.Add(geo);

            Curve newTrail = birds.birdsList[i].drawTrail(birds.birdsList[i].trail);
            if (newTrail != null) boidsTrailsList.Add(newTrail);

            Curve oldTrail = birds.birdsList[i].drawTrail(birds.birdsList[i].trailOld);
            if (oldTrail != null) boidsTrailsList.Add(oldTrail);
        }

        boidsLocations = boidsLocationsList;
        boidsTrails    = boidsTrailsList;
        BoidDirections = boidDirectionsList;
        BoidsGeometry  = boidsGeoList;



        PointCloud pc = new PointCloud();
        if (cfdPoints != null && cfdVectors != null
            && cfdPoints.Count > 0
            && cfdPoints.Count == cfdVectors.Count)
        {
        double minMag = double.MaxValue;
        double maxMag = double.MinValue;
        double[] mags = new double[cfdVectors.Count];
        for (int i = 0; i < cfdVectors.Count; i++)
            {
                mags[i] = cfdVectors[i].Length;
                if (mags[i] < minMag) minMag = mags[i];
                if (mags[i] > maxMag) maxMag = mags[i];
            }
            double range = maxMag - minMag;
            if (range < 1e-9) range = 1.0;

            for (int i = 0; i < cfdPoints.Count; i++)
            {
                Point3d worldPt = cfdPoints[i];
                worldPt.Transform(cfdXform);
                double t = (mags[i] - minMag) / range;
                pc.Add(worldPt, MagnitudeColor(t));
            }
        }
        cfdCloud = pc;
    }


    //blue low, red high
    static Color MagnitudeColor(double t)
    {
        if (t < 0) t = 0; if (t > 1) t = 1;
        double hue = (1.0 - t) * 240.0;
        return ColorFromHSV(hue, 1.0, 1.0);
    }

    static Color ColorFromHSV(double h, double s, double v)
    {
        h = ((h % 360) + 360) % 360; //normalise to 360
        int hi = (int)Math.Floor(h/60)%6;
        double f = h /60 -Math.Floor(h/60);

        double p = v * (1-s);
        double q = v * (1-f*s);
        double t = v * (1-(1- f)*s);

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

        return Color.FromArgb
            (255, 
            (int)Math.Round(rgb[0]*255), 
            (int)Math.Round(rgb[1]*255), 
            (int)Math.Round(rgb[2]*255)
            );
    }


//cfd
//incoming query , inverse transform, grid look up, thus movement vectors dont require rebuild
    public class CFDField
    {
        private double[]   xs, ys, zs;
        private Vector3d[,,] vectors;
        private Transform forwardXform;
        private Transform inverseXform;
        private bool      hasInverse = false;
        public  bool      isValid = false;

        private const double COORD_TOL = 1e-4;

        public CFDField(List<Point3d> points, List<Vector3d> vecs, Transform xform)
        {
            UpdateTransform(xform);

            if (points.Count == 0) return;

            xs = UniqueSorted(points.Select(p => p.X));
            ys = UniqueSorted(points.Select(p => p.Y));
            zs = UniqueSorted(points.Select(p => p.Z));

            int nx = xs.Length, ny = ys.Length, nz = zs.Length;

            if ((double)nx * ny * nz != points.Count) return;

            vectors = new Vector3d[nx, ny, nz];
            for (int p = 0; p < points.Count; p++)
            {
                int i = IndexOf(xs, points[p].X);
                int j = IndexOf(ys, points[p].Y);
                int k = IndexOf(zs, points[p].Z);
                if (i < 0 || j < 0 || k < 0) return;
                vectors[i, j, k] = vecs[p];
            }

            isValid = true;
        }



        public void UpdateTransform(Transform xform)
        {
            forwardXform = xform;
            Transform inv;
            hasInverse = xform.TryGetInverse(out inv);
            inverseXform = inv;
        }



        private static double[] UniqueSorted(IEnumerable<double> values)
        {
            List<double> sorted = values.OrderBy(v => v).ToList();
            List<double> unique = new List<double>();
            foreach (double v in sorted)
            {
                if (unique.Count == 0 || Math.Abs(v - unique[unique.Count - 1]) > COORD_TOL)
                unique.Add(v);
            }
            return unique.ToArray();
        }

        private static int IndexOf(double[] arr, double v)
        {
            int lo = 0, hi = arr.Length - 1;
            while (lo <= hi)
            {
                int mid = (lo + hi) >> 1;
                if (Math.Abs(arr[mid] - v) <= COORD_TOL) return mid;
                if (arr[mid] < v) lo = mid + 1; else hi = mid - 1;
            }
            return -1;
        }

        private static int LowerIndex(double[] arr, double v)
        {
            if (v < arr[0]) return -1;
            if (v >= arr[arr.Length - 1]) return arr.Length - 1;

            int lo = 0, hi = arr.Length - 1;
            while (lo < hi - 1)
            {
                int mid = (lo + hi) >> 1;
                if (arr[mid] <= v) lo = mid; else hi = mid;
            }
            return lo;
        }

        public Vector3d Sample(Point3d worldPos)
        {
            if (!isValid || !hasInverse) return Vector3d.Zero;

            //map boid to cfd grid local coord
            Point3d local = worldPos;
            local.Transform(inverseXform);

            int i = LowerIndex(xs, local.X);
            int j = LowerIndex(ys, local.Y);
            int k = LowerIndex(zs, local.Z);

            if (i < 0 || j < 0 || k < 0)              return Vector3d.Zero;
            if (i >= xs.Length - 1)                   return Vector3d.Zero;
            if (j >= ys.Length - 1)                   return Vector3d.Zero;
            if (k >= zs.Length - 1)                   return Vector3d.Zero;

            double tx = (local.X - xs[i]) / (xs[i + 1] - xs[i]);
            double ty = (local.Y - ys[j]) / (ys[j + 1] - ys[j]);
            double tz = (local.Z - zs[k]) / (zs[k + 1] - zs[k]);

            Vector3d c000 = vectors[i,     j,     k    ];
            Vector3d c100 = vectors[i + 1, j,     k    ];
            Vector3d c010 = vectors[i,     j + 1, k    ];
            Vector3d c110 = vectors[i + 1, j + 1, k    ];
            Vector3d c001 = vectors[i,     j,     k + 1];
            Vector3d c101 = vectors[i + 1, j,     k + 1];
            Vector3d c011 = vectors[i,     j + 1, k + 1];
            Vector3d c111 = vectors[i + 1, j + 1, k + 1];

            Vector3d c00 = c000 * (1 - tx) + c100 * tx;
            Vector3d c10 = c010 * (1 - tx) + c110 * tx;
            Vector3d c01 = c001 * (1 - tx) + c101 * tx;
            Vector3d c11 = c011 * (1 - tx) + c111 * tx;
            Vector3d c0  = c00  * (1 - ty) + c10  * ty;
            Vector3d c1  = c01  * (1 - ty) + c11  * ty;

            //rotate back to world coords
            Vector3d sampled = c0 * (1 - tz) + c1 * tz;
            sampled.Transform(forwardXform);
            return sampled;
        }
    }


    // -----------------------------------------------------------------------
    //  Spatial hash for boid neighbour queries.
    //  Bins boids into cubic cells of side = the largest query radius, so the
    //  3x3x3 block around any boid is guaranteed to contain every neighbour
    //  within that radius. Replaces the O(n^2) all-pairs scan with ~O(n*k).
    // -----------------------------------------------------------------------
    struct CellKey : IEquatable<CellKey>
    {
        public readonly int X, Y, Z;
        public CellKey(int x, int y, int z) { X = x; Y = y; Z = z; }
        public bool Equals(CellKey o) { return X == o.X && Y == o.Y && Z == o.Z; }
        public override bool Equals(object o) { return o is CellKey && Equals((CellKey)o); }
        public override int GetHashCode()
        {
            unchecked
            {
                int h = 17;
                h = h * 31 + X;
                h = h * 31 + Y;
                h = h * 31 + Z;
                return h;
            }
        }
    }

    public class SpatialHash
    {
        private readonly double cellSize;
        private readonly Dictionary<CellKey, List<int>> buckets = new Dictionary<CellKey, List<int>>();
        private readonly CellKey[] cellOf; // current cell of each boid index

        public SpatialHash(double cellSize, int boidCount)
        {
            this.cellSize = cellSize <= 1e-9 ? 1.0 : cellSize;
            cellOf = new CellKey[boidCount];
        }

        private CellKey CellAt(Point3d p)
        {
            return new CellKey(
                (int)Math.Floor(p.X / cellSize),
                (int)Math.Floor(p.Y / cellSize),
                (int)Math.Floor(p.Z / cellSize));
        }

        private void AddTo(CellKey key, int index)
        {
            List<int> list;
            if (!buckets.TryGetValue(key, out list))
            {
                list = new List<int>();
                buckets[key] = list;
            }
            list.Add(index);
        }

        public void Insert(int index, Point3d p)
        {
            CellKey key = CellAt(p);
            cellOf[index] = key;
            AddTo(key, index);
        }

        // Re-bin a boid after it has moved (keeps the structure live within a frame).
        public void UpdatePosition(int index, Point3d p)
        {
            CellKey newKey = CellAt(p);
            CellKey oldKey = cellOf[index];
            if (newKey.Equals(oldKey)) return;

            List<int> old;
            if (buckets.TryGetValue(oldKey, out old)) old.Remove(index);

            cellOf[index] = newKey;
            AddTo(newKey, index);
        }

        // Fill 'results' with candidate boid indices in the 3x3x3 block around p.
        public void Query(Point3d p, List<int> results)
        {
            results.Clear();
            CellKey c = CellAt(p);
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        CellKey key = new CellKey(c.X + dx, c.Y + dy, c.Z + dz);
                        List<int> list;
                        if (buckets.TryGetValue(key, out list))
                            results.AddRange(list);
                    }
        }
    }


    public class Flock
    {
        public List<Boid> birdsList;

        // Spatial hash + a reusable candidate buffer (rebuilt each UpdateFlock).
        private SpatialHash hash;
        private List<int> candidateBuffer = new List<int>();

        public Flock(int numberOfBoids, Point3d starPoint)
        {
            birdsList = new List<Boid>();
            rnd = new Random();
            for (int i = 0; i < numberOfBoids; i++)
            {
                double vx = (rnd.NextDouble() * 2.0) - 1.0;
                double vy = (rnd.NextDouble() * 2.0) - 1.0;
                double vz = (rnd.NextDouble() * 2.0) - 1.0;
                birdsList.Add(new Boid(starPoint, new Vector3d(vx, vy, vz)));
            }
        }

        public Flock(int numberOfBoids, Point3d[] starPoints)
        {
            rnd = new Random();
            birdsList = new List<Boid>();
            for (int i = 0; i < numberOfBoids; i++)
            {
                double vx = (rnd.NextDouble() * 2.0) - 1.0;
                double vy = (rnd.NextDouble() * 2.0) - 1.0;
                double vz = (rnd.NextDouble() * 2.0) - 1.0;
                birdsList.Add(new Boid(starPoints[i], new Vector3d(vx, vy, vz)));
            }
        }

        public void UpdateFlock(double speed, double alignmentFactor, 
        double cohereFactor, double separationFactor, 
        double desiredSeparation, double neighbourhoodDist)

        {
            // Cell size = largest query radius, so the 3x3x3 block covers every
            // neighbour within neighbourhoodDist AND within desiredSeparation.
            double cell = Math.Max(neighbourhoodDist, desiredSeparation);
            hash = new SpatialHash(cell, birdsList.Count);
            for (int i = 0; i < birdsList.Count; i++)
                hash.Insert(i, birdsList[i].location);

            for (int i = 0; i < birdsList.Count; i++)
            steerBoid(i, speed, alignmentFactor, cohereFactor, separationFactor, desiredSeparation, neighbourhoodDist);
        }

        void steerBoid(int boidIndex, 
        double speed, double alignmentFactor, double cohereFactor, 
        double separationFactor, double desiredSeparation, double neighbourhoodDist)

        {
            // One neighbourhood query per boid, shared by all three rules.
            // Sorting by index reproduces the original ascending-order summation,
            // so results are identical to the brute-force loop.
            hash.Query(birdsList[boidIndex].location, candidateBuffer);
            candidateBuffer.Sort();

            Vector3d combined = alignment(alignmentFactor, boidIndex, neighbourhoodDist, candidateBuffer)
            + separation(separationFactor, boidIndex, desiredSeparation, candidateBuffer)
            + cohere(cohereFactor, boidIndex, neighbourhoodDist, candidateBuffer)
            + attractorField(boidIndex);

            if (Script_Instance.cfdField != null && Script_Instance.cfdField.isValid)
            {
                Vector3d wind = Script_Instance.cfdField.Sample(birdsList[boidIndex].location);
                combined += wind * Script_Instance.windFactor;
            }

            Vector3d velocityVec = birdsList[boidIndex].direction + combined;
            velocityVec.Unitize();
            birdsList[boidIndex].direction = velocityVec;
            birdsList[boidIndex].updateLocation(speed);

            // Keep the hash live so later boids this frame see the moved position.
            hash.UpdatePosition(boidIndex, birdsList[boidIndex].location);
        }



        Vector3d alignment(double alignmentFactor, int boidIndex, double neighbourhoodDist, List<int> candidates)
        {
            Vector3d alignmentVec = new Vector3d();
            for (int idx = 0; idx < candidates.Count; idx++)
            {
                int i = candidates[idx];
                if (i == boidIndex) continue;
                if (birdsList[boidIndex].location.DistanceTo(birdsList[i].location) < neighbourhoodDist)
                alignmentVec += birdsList[i].direction;
            }
            alignmentVec.Unitize();
            return alignmentVec * alignmentFactor;
        }

        Vector3d cohere(double cohereFactor, int boidIndex, double neighbourhoodDist, List<int> candidates)
        {
            Point3d center = new Point3d();
            int count = 0;
            for (int idx = 0; idx < candidates.Count; idx++)
            {
                int i = candidates[idx];
                if (i == boidIndex) continue;
                if (birdsList[boidIndex].location.DistanceTo(birdsList[i].location) < neighbourhoodDist)
                {
                    center += birdsList[i].location;
                    count++;
                }
            }
            if (count == 0) return new Vector3d(0, 0, 0);
            center = center / count;
            Vector3d cohereVec = center - birdsList[boidIndex].location;
            cohereVec.Unitize();
            return cohereVec * cohereFactor;
        }



        Vector3d separation(double separationFactor, int boidIndex, double desiredSeparation, List<int> candidates)
        {
            Vector3d sepVec = new Vector3d();
            for (int idx = 0; idx < candidates.Count; idx++)
            {
                int i = candidates[idx];
                if (i == boidIndex) continue;
                double dist = birdsList[boidIndex].location.DistanceTo(birdsList[i].location);
                if (dist > 0 && dist < desiredSeparation)
                sepVec += new Vector3d(birdsList[boidIndex].location - birdsList[i].location) * (1.0 / dist);
            }
            sepVec.Unitize();
            return sepVec * separationFactor;
        }

//f0r every pair, build unit direction vector, scale, accumulate and multiply by gain 
//attractor 
        Vector3d attractorField(int boidIndex)
        {
            List<Point3d> pts  = Script_Instance.attractorPts;
            List<double>  vals = Script_Instance.attractorVals;
            double        gain = Script_Instance.attractorGain;

            if (pts == null || pts.Count == 0 || gain == 0) return Vector3d.Zero;

            Vector3d sum = Vector3d.Zero;
            Point3d  loc = birdsList[boidIndex].location;

            for (int j = 0; j < pts.Count; j++)
            {
                double w = vals[j];
                if (w == 0) continue;

                Vector3d desired = new Vector3d(pts[j]) - new Vector3d(loc);
                if (desired.Length < 1e-9) continue;
                desired.Unitize();
                sum += desired * w;
            }
            return sum * gain;
        }



        public void stayInBoundery(bool wrap)
        {
            Brep bounds = Script_Instance.boundingVolume;
            if (bounds == null) return;

            for (int i = 0; i < birdsList.Count; i++)
            {
                Point3d loc = birdsList[i].location;

//obstacle
                bool hitObstacle = false;
                foreach (Brep obs in Script_Instance.obstacleVolumes)
                {
                    if (obs == null) continue;
                    if (!obs.IsPointInside(loc, RhinoMath.SqrtEpsilon, false)) continue;

                    double s, t;
                    ComponentIndex ci;
                    Point3d cp;
                    Vector3d normal;
                    obs.ClosestPoint(loc, out cp, out ci, out s, out t, 0.0, out normal);
                    normal.Unitize();

                    birdsList[i].location = cp + normal * RhinoMath.SqrtEpsilon * 10;
                    Vector3d d = birdsList[i].direction;
                    birdsList[i].direction = d - 2.0 * (d * normal) * normal;
                    birdsList[i].direction.Unitize();

                    hitObstacle = true;
                    break;
                }
                if (hitObstacle) continue;


// bounding volume check
                if (bounds.IsPointInside(loc, RhinoMath.SqrtEpsilon, false)) continue;

                double bs, bt;
                ComponentIndex bci;
                Point3d bcp;
                Vector3d bnormal;
                bounds.ClosestPoint(loc, out bcp, out bci, out bs, out bt, 0.0, out bnormal);
                bnormal.Unitize();

                if (wrap)
                {
                    BoundingBox bb = bounds.GetBoundingBox(true);
                    Point3d p = birdsList[i].location;

                    double x = p.X;
                    double y = p.Y;
                    double z = p.Z;

                    if (x < bb.Min.X) x = bb.Max.X - (bb.Min.X - x);
                    else if (x > bb.Max.X) x = bb.Min.X + (x - bb.Max.X);

                    if (y < bb.Min.Y) y = bb.Max.Y - (bb.Min.Y - y);
                    else if (y > bb.Max.Y) y = bb.Min.Y + (y - bb.Max.Y);

                    if (z < bb.Min.Z) z = bb.Max.Z - (bb.Min.Z - z);
                    else if (z > bb.Max.Z) z = bb.Min.Z + (z - bb.Max.Z);

                    if (x != p.X || y != p.Y || z != p.Z)
                    {
                        birdsList[i].location = new Point3d(x, y, z);
                        birdsList[i].trailOld = birdsList[i].trail;
                        birdsList[i].trail    = new List<Point3d>();
                    }
                }


                else
                {
                    Vector3d inward = -bnormal;
                    birdsList[i].location = bcp + inward * RhinoMath.SqrtEpsilon * 10;
                    Vector3d d = birdsList[i].direction;
                    birdsList[i].direction = d - 2.0 * (d * bnormal) * bnormal;
                    birdsList[i].direction.Unitize();
                }
            }
        }

        public void checkTeleport()
        {
            List<Curve> curvesA = Script_Instance.teleportCurvesA;
            List<Curve> curvesB = Script_Instance.teleportCurvesB;
            double radius       = Script_Instance.teleportRadius;

            int pairCount = Math.Min(curvesA.Count, curvesB.Count);
            if (pairCount == 0) return;

            for (int i = 0; i < birdsList.Count; i++)
            {
                if (birdsList[i].justTeleported)
                {
                    birdsList[i].justTeleported = false;
                    continue;
                }

                Point3d loc = birdsList[i].location;

                for (int p = 0; p < pairCount; p++)
                {
                    Curve cA = curvesA[p];
                    Curve cB = curvesB[p];
                    if (cA == null || cB == null) continue;

                    double tA;
                    cA.ClosestPoint(loc, out tA);
                    if (loc.DistanceTo(cA.PointAt(tA)) < radius)
                    {
                        TeleportBoid(i, cA, tA, cB);
                        break;
                    }

                    double tB;
                    cB.ClosestPoint(loc, out tB);
                    if (loc.DistanceTo(cB.PointAt(tB)) < radius)
                    {
                        TeleportBoid(i, cB, tB, cA);
                        break;
                    }
                }
            }
        }

        void TeleportBoid(int i, Curve sourceCurve, double t, Curve destCurve)
        {
            double tNorm = (t - sourceCurve.Domain.Min) / sourceCurve.Domain.Length;
            double tDest = destCurve.Domain.Min + tNorm * destCurve.Domain.Length;

            Point3d destPt = destCurve.PointAt(tDest);

            Plane srcFrame, dstFrame;
            sourceCurve.FrameAt(t, out srcFrame);
            destCurve.FrameAt(tDest, out dstFrame);

            Transform remap = Transform.PlaneToPlane(srcFrame, dstFrame);
            Vector3d newDir = birdsList[i].direction;
            newDir.Transform(remap);
            newDir.Unitize();

            birdsList[i].trailOld = birdsList[i].trail;
            birdsList[i].trail = new List<Point3d>();
            birdsList[i].location = destPt;
            birdsList[i].direction = newDir;
            birdsList[i].justTeleported = true;
        }
    }


    public class Boid
    {
        public Point3d location;
        public Vector3d direction;
        public List<Point3d> trail;
        public List<Point3d> trailOld;
        public Vector3d boidNormal;
        public bool justTeleported = false;

        public Boid(Point3d startlocation, Vector3d startDirection)
        {
            trail = new List<Point3d>();
            trailOld = new List<Point3d>();
            location = startlocation;
            direction = startDirection;
        }

        public void updateLocation(double speed)
        {
            location = location + (new Point3d(direction * speed));
            trail.Add(location);
            if (trail.Count + trailOld.Count > trailLength)
            {
                if (trailOld.Count> 0)
                trailOld.RemoveAt(0);
                else
                trail.RemoveAt(0);
            }
        }

        public Curve drawTrail(List<Point3d> pts)
        {
            if (pts.Count < 2) return null;
            return Curve.CreateInterpolatedCurve(pts, 3);
        }

        public Point3d oldLoc()
        {
            if (trail.Count > 1) return trail[1];
            return Point3d.Unset;
        }

        public Brep drawGeo(Brep boidGeo)
        {
            if (boidGeo == null) return null;
            Brep tranBrep = boidGeo.DuplicateBrep();
            Plane plane2 = new Plane(location, direction);
            Transform orient = Transform.PlaneToPlane(Plane.WorldXY, plane2);
            tranBrep.Transform(orient);
            return tranBrep;
        }
    }
}
