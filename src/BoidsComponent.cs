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
    //  Boids — 3D flocking with alignment/cohesion/separation, an attractor
    //  field, obstacle bouncing, curve-to-curve teleport portals, optional
    //  CFD wind sampling, and a coloured CFD magnitude cloud.
    //
    //  Stepper: advances one frame per solve. Drive it with a Timer (or any
    //  recompute trigger). Set Reset to re-seed the flock.
    // -----------------------------------------------------------------------
    public class BoidsComponent : GH_Component
    {
        // Simulation state persists across solves (the component instance lives
        // for the lifetime of the document, so these survive between frames).
        private Flock _birds;

        public BoidsComponent()
          : base("Boids", "Boids",
                 "3D flocking simulation with attractor field, obstacles, teleport portals " +
                 "and optional CFD wind. Advances one frame per solve; drive with a Timer.",
                 "Geoboid", "Geoboid")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Reset", "Reset", "Re-seed the flock from scratch.", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Wrap", "Wrap", "Wrap across the bounding volume instead of bouncing.", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Cut Trails", "Cut", "Reserved trail-cutting flag (currently unused).", GH_ParamAccess.item, true);
            pManager.AddIntegerParameter("Count", "N", "Number of boids (used on reset).", GH_ParamAccess.item, 100);
            pManager.AddNumberParameter("Speed", "Spd", "Step length per frame.", GH_ParamAccess.item, 0.5);
            pManager.AddIntegerParameter("Trail Length", "TL", "Max trail point count per boid.", GH_ParamAccess.item, 50);
            pManager.AddNumberParameter("Neighbourhood", "Nbr", "Neighbour search distance.", GH_ParamAccess.item, 5.0);
            pManager.AddNumberParameter("Cohesion", "Coh", "Cohesion weight.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Alignment", "Alg", "Alignment weight.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Separation", "Sep", "Separation weight.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Separation Dist", "SepD", "Desired separation distance.", GH_ParamAccess.item, 2.0);
            pManager.AddPointParameter("Attractor Points", "AP", "Attractor/detractor points.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Attractor Values", "AV", "Per-point attractor weights (parallel to Attractor Points).", GH_ParamAccess.list);
            pManager.AddNumberParameter("Attractor Weight", "AW", "Global attractor gain.", GH_ParamAccess.item, 1.0);
            pManager.AddBrepParameter("Bounding Volume", "BV", "Closed Brep the flock stays inside.", GH_ParamAccess.item);
            pManager.AddBrepParameter("Boid Geometry", "BG", "Brep oriented to each boid when Show Boids is true.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Show Boids", "Show", "Output oriented boid geometry.", GH_ParamAccess.item, false);
            pManager.AddBrepParameter("Obstacles", "Obs", "Closed Breps the flock bounces off.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Teleport A", "TpA", "Source curves for teleport portals.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Teleport B", "TpB", "Destination curves (paired by index with Teleport A).", GH_ParamAccess.list);
            pManager.AddPointParameter("CFD Points", "CP", "CFD grid sample points.", GH_ParamAccess.list);
            pManager.AddVectorParameter("CFD Vectors", "CV", "CFD velocity vectors (parallel to CFD Points).", GH_ParamAccess.list);
            pManager.AddVectorParameter("CFD Move", "CM", "Translation applied to the CFD field.", GH_ParamAccess.item, Vector3d.Zero);
            pManager.AddNumberParameter("CFD Rotation", "CR", "CFD field rotation in degrees about world Z.", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Wind Factor", "Wind", "Scales the sampled CFD wind influence on the boids.", GH_ParamAccess.item, 0.0);

            // Everything except the bounding volume is optional.
            int[] optional = { 11, 12, 15, 17, 18, 19, 20, 21 };
            foreach (int i in optional) pManager[i].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Trails", "T", "Boid trail curves (current + old segments).", GH_ParamAccess.list);
            pManager.AddPointParameter("Locations", "L", "Boid positions this frame.", GH_ParamAccess.list);
            pManager.AddVectorParameter("Directions", "D", "Boid unit directions this frame.", GH_ParamAccess.list);
            pManager.AddBrepParameter("Geometry", "G", "Oriented boid geometry (when Show Boids is true).", GH_ParamAccess.list);
            pManager.AddGenericParameter("CFD Cloud", "C", "Colour-mapped CFD magnitude point cloud.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool reset = false, wrap = false, cutTrails = true, showBoids = false;
            int num = 100, trailLen = 50;
            double speed = 0.5, niegh = 5.0, coh = 1.0, alig = 1.0, sep = 1.0, sepDist = 2.0;
            double attractorWeight = 1.0, cfdRotAngle = 0.0, windF = 0.0;
            Vector3d cfdMoveVec = Vector3d.Zero;
            Brep boundingBox = null, boidG = null;

            var attractorPoints = new List<Point3d>();
            var attractorValues = new List<double>();
            var obstacles = new List<Brep>();
            var teleportA = new List<Curve>();
            var teleportB = new List<Curve>();
            var cfdPoints = new List<Point3d>();
            var cfdVectors = new List<Vector3d>();

            DA.GetData(0, ref reset);
            DA.GetData(1, ref wrap);
            DA.GetData(2, ref cutTrails);
            DA.GetData(3, ref num);
            DA.GetData(4, ref speed);
            DA.GetData(5, ref trailLen);
            DA.GetData(6, ref niegh);
            DA.GetData(7, ref coh);
            DA.GetData(8, ref alig);
            DA.GetData(9, ref sep);
            DA.GetData(10, ref sepDist);
            DA.GetDataList(11, attractorPoints);
            DA.GetDataList(12, attractorValues);
            DA.GetData(13, ref attractorWeight);
            DA.GetData(14, ref boundingBox);
            DA.GetData(15, ref boidG);
            DA.GetData(16, ref showBoids);
            DA.GetDataList(17, obstacles);
            DA.GetDataList(18, teleportA);
            DA.GetDataList(19, teleportB);
            DA.GetDataList(20, cfdPoints);
            DA.GetDataList(21, cfdVectors);
            DA.GetData(22, ref cfdMoveVec);
            DA.GetData(23, ref cfdRotAngle);
            DA.GetData(24, ref windF);

            if (boundingBox == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No bounding volume supplied.");
                return;
            }

            BoundingBox bb = boundingBox.GetBoundingBox(true);

            // CFD field transform: rotate about world Z at origin, then translate.
            Transform cfdXform = Transform.Identity;
            if (Math.Abs(cfdRotAngle) > 1e-12)
            {
                double rad = cfdRotAngle * Math.PI / 180.0;
                cfdXform = Transform.Rotation(rad, Vector3d.ZAxis, Point3d.Origin);
            }
            if (cfdMoveVec.Length > 1e-12)
                cfdXform = Transform.Translation(cfdMoveVec) * cfdXform;

            bool cfdValid = cfdPoints.Count > 0 && cfdPoints.Count == cfdVectors.Count;

            // Build / re-seed the flock on reset.
            if (reset || _birds == null)
            {
                Random rnd = new Random();
                Point3d[] rndStartPoints = new Point3d[num];
                for (int i = 0; i < num; i++)
                {
                    rndStartPoints[i] = new Point3d(
                        bb.Min.X + rnd.NextDouble() * (bb.Max.X - bb.Min.X),
                        bb.Min.Y + rnd.NextDouble() * (bb.Max.Y - bb.Min.Y),
                        bb.Min.Z + rnd.NextDouble() * (bb.Max.Z - bb.Min.Z));
                }
                _birds = new Flock(num, rndStartPoints);

                _birds.cfdField = cfdValid ? new CFDField(cfdPoints, cfdVectors, cfdXform) : null;
            }
            else if (_birds.cfdField != null)
            {
                // Push the new transform without rebuilding the grid.
                _birds.cfdField.UpdateTransform(cfdXform);
            }

            // Refresh the flock's context every frame (these can change without a reset).
            _birds.boundingVolume = boundingBox;
            _birds.obstacleVolumes = obstacles;
            _birds.teleportCurvesA = teleportA;
            _birds.teleportCurvesB = teleportB;
            _birds.windFactor = windF;
            _birds.attractorPts = attractorPoints;
            _birds.attractorVals = attractorValues;
            _birds.attractorGain = attractorWeight;
            _birds.trailLength = trailLen;
            _birds.cutTrails = cutTrails;

            // Step the simulation.
            _birds.UpdateFlock(speed, alig, coh, sep, sepDist, niegh);
            _birds.stayInBoundery(wrap);
            _birds.checkTeleport();

            // Gather outputs.
            var locations = new List<Point3d>();
            var trails = new List<Curve>();
            var directions = new List<Vector3d>();
            var geometry = new List<Brep>();

            for (int i = 0; i < _birds.birdsList.Count; i++)
            {
                Boid b = _birds.birdsList[i];
                locations.Add(b.location);
                directions.Add(b.direction);

                if (showBoids)
                {
                    Brep geo = b.drawGeo(boidG);
                    if (geo != null) geometry.Add(geo);
                }

                Curve newTrail = b.drawTrail(b.trail);
                if (newTrail != null) trails.Add(newTrail);

                Curve oldTrail = b.drawTrail(b.trailOld);
                if (oldTrail != null) trails.Add(oldTrail);
            }

            // CFD magnitude cloud (independent of the wind sampler above).
            var pc = new PointCloud();
            if (cfdValid)
            {
                double minMag = double.MaxValue, maxMag = double.MinValue;
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

            DA.SetDataList(0, trails);
            DA.SetDataList(1, locations);
            DA.SetDataList(2, directions);
            DA.SetDataList(3, geometry);
            DA.SetData(4, pc);
        }

        // ---- colour helpers (unchanged) ----

        private static Color MagnitudeColor(double t) // blue low, red high
        {
            if (t < 0) t = 0; if (t > 1) t = 1;
            double hue = (1.0 - t) * 240.0;
            return ColorFromHSV(hue, 1.0, 1.0);
        }

        private static Color ColorFromHSV(double h, double s, double v)
        {
            h = ((h % 360) + 360) % 360;
            int hi = (int)Math.Floor(h / 60) % 6;
            double f = h / 60 - Math.Floor(h / 60);

            double p = v * (1 - s);
            double q = v * (1 - f * s);
            double t = v * (1 - (1 - f) * s);

            double[][] sectors = new double[][]
            {
                new[] { v, t, p },
                new[] { q, v, p },
                new[] { p, v, t },
                new[] { p, q, v },
                new[] { t, p, v },
                new[] { v, p, q }
            };

            double[] rgb = sectors[hi];
            return Color.FromArgb(255,
                (int)Math.Round(rgb[0] * 255),
                (int)Math.Round(rgb[1] * 255),
                (int)Math.Round(rgb[2] * 255));
        }

        // =======================================================================
        //  CFD field: query -> inverse transform -> grid lookup -> trilinear.
        //  Movement vectors don't require a rebuild when the transform changes.
        // =======================================================================
        public class CFDField
        {
            private double[] xs, ys, zs;
            private Vector3d[,,] vectors;
            private Transform forwardXform;
            private Transform inverseXform;
            private bool hasInverse = false;
            public bool isValid = false;

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

                // Map the boid into the CFD grid's local coordinates.
                Point3d local = worldPos;
                local.Transform(inverseXform);

                int i = LowerIndex(xs, local.X);
                int j = LowerIndex(ys, local.Y);
                int k = LowerIndex(zs, local.Z);

                if (i < 0 || j < 0 || k < 0) return Vector3d.Zero;
                if (i >= xs.Length - 1) return Vector3d.Zero;
                if (j >= ys.Length - 1) return Vector3d.Zero;
                if (k >= zs.Length - 1) return Vector3d.Zero;

                double tx = (local.X - xs[i]) / (xs[i + 1] - xs[i]);
                double ty = (local.Y - ys[j]) / (ys[j + 1] - ys[j]);
                double tz = (local.Z - zs[k]) / (zs[k + 1] - zs[k]);

                Vector3d c000 = vectors[i, j, k];
                Vector3d c100 = vectors[i + 1, j, k];
                Vector3d c010 = vectors[i, j + 1, k];
                Vector3d c110 = vectors[i + 1, j + 1, k];
                Vector3d c001 = vectors[i, j, k + 1];
                Vector3d c101 = vectors[i + 1, j, k + 1];
                Vector3d c011 = vectors[i, j + 1, k + 1];
                Vector3d c111 = vectors[i + 1, j + 1, k + 1];

                Vector3d c00 = c000 * (1 - tx) + c100 * tx;
                Vector3d c10 = c010 * (1 - tx) + c110 * tx;
                Vector3d c01 = c001 * (1 - tx) + c101 * tx;
                Vector3d c11 = c011 * (1 - tx) + c111 * tx;
                Vector3d c0 = c00 * (1 - ty) + c10 * ty;
                Vector3d c1 = c01 * (1 - ty) + c11 * ty;

                // Rotate the sampled vector back into world coordinates.
                Vector3d sampled = c0 * (1 - tz) + c1 * tz;
                sampled.Transform(forwardXform);
                return sampled;
            }
        }

        // =======================================================================
        //  Flock: owns the boids and all the per-frame context that used to live
        //  in static fields.
        // =======================================================================
        public class Flock
        {
            public List<Boid> birdsList;

            // Context (set by the component each frame).
            public Brep boundingVolume;
            public List<Brep> obstacleVolumes = new List<Brep>();
            public List<Curve> teleportCurvesA = new List<Curve>();
            public List<Curve> teleportCurvesB = new List<Curve>();
            public double windFactor = 0.0;
            public List<Point3d> attractorPts = new List<Point3d>();
            public List<double> attractorVals = new List<double>();
            public double attractorGain = 1.0;
            public int trailLength = 0;
            public bool cutTrails = true; // currently unused, kept for parity
            public CFDField cfdField = null;

            private const double teleportRadius = 0.1; // shrink if teleporting too early

            public Flock(int numberOfBoids, Point3d[] starPoints)
            {
                Random rnd = new Random();
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
                for (int i = 0; i < birdsList.Count; i++)
                    steerBoid(i, speed, alignmentFactor, cohereFactor, separationFactor, desiredSeparation, neighbourhoodDist);
            }

            void steerBoid(int boidIndex,
                double speed, double alignmentFactor, double cohereFactor,
                double separationFactor, double desiredSeparation, double neighbourhoodDist)
            {
                Vector3d combined = alignment(alignmentFactor, boidIndex, neighbourhoodDist)
                    + separation(separationFactor, boidIndex, desiredSeparation)
                    + cohere(cohereFactor, boidIndex, neighbourhoodDist)
                    + attractorField(boidIndex);

                if (cfdField != null && cfdField.isValid)
                {
                    Vector3d wind = cfdField.Sample(birdsList[boidIndex].location);
                    combined += wind * windFactor;
                }

                Vector3d velocityVec = birdsList[boidIndex].direction + combined;
                velocityVec.Unitize();
                birdsList[boidIndex].direction = velocityVec;
                birdsList[boidIndex].updateLocation(speed, trailLength);
            }

            Vector3d alignment(double alignmentFactor, int boidIndex, double neighbourhoodDist)
            {
                Vector3d alignmentVec = new Vector3d();
                for (int i = 0; i < birdsList.Count; i++)
                {
                    if (i == boidIndex) continue;
                    if (birdsList[boidIndex].location.DistanceTo(birdsList[i].location) < neighbourhoodDist)
                        alignmentVec += birdsList[i].direction;
                }
                alignmentVec.Unitize();
                return alignmentVec * alignmentFactor;
            }

            Vector3d cohere(double cohereFactor, int boidIndex, double neighbourhoodDist)
            {
                Point3d center = new Point3d();
                int count = 0;
                for (int i = 0; i < birdsList.Count; i++)
                {
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

            Vector3d separation(double separationFactor, int boidIndex, double desiredSeparation)
            {
                Vector3d sepVec = new Vector3d();
                for (int i = 0; i < birdsList.Count; i++)
                {
                    if (i == boidIndex) continue;
                    double dist = birdsList[boidIndex].location.DistanceTo(birdsList[i].location);
                    if (dist > 0 && dist < desiredSeparation)
                        sepVec += new Vector3d(birdsList[boidIndex].location - birdsList[i].location) * (1.0 / dist);
                }
                sepVec.Unitize();
                return sepVec * separationFactor;
            }

            // For each attractor: unit direction toward it, scaled by its value,
            // accumulated and multiplied by the global gain.
            Vector3d attractorField(int boidIndex)
            {
                List<Point3d> pts = attractorPts;
                List<double> vals = attractorVals;
                double gain = attractorGain;

                if (pts == null || pts.Count == 0 || gain == 0) return Vector3d.Zero;

                Vector3d sum = Vector3d.Zero;
                Point3d loc = birdsList[boidIndex].location;

                for (int j = 0; j < pts.Count; j++)
                {
                    if (j >= vals.Count) break; // guard parallel-list mismatch
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
                Brep bounds = boundingVolume;
                if (bounds == null) return;

                for (int i = 0; i < birdsList.Count; i++)
                {
                    Point3d loc = birdsList[i].location;

                    // Obstacles first.
                    bool hitObstacle = false;
                    foreach (Brep obs in obstacleVolumes)
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

                    // Bounding volume.
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

                        double x = p.X, y = p.Y, z = p.Z;

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
                            birdsList[i].trail = new List<Point3d>();
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
                List<Curve> curvesA = teleportCurvesA;
                List<Curve> curvesB = teleportCurvesB;
                double radius = teleportRadius;

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

        // =======================================================================
        //  Boid
        // =======================================================================
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

            public void updateLocation(double speed, int trailLength)
            {
                location = location + (new Point3d(direction * speed));
                trail.Add(location);
                if (trail.Count + trailOld.Count > trailLength)
                {
                    if (trailOld.Count > 0) trailOld.RemoveAt(0);
                    else trail.RemoveAt(0);
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

        protected override Bitmap Icon =>
            new Bitmap(GetType().Assembly.GetManifestResourceStream("MyPlugin.Icons.Boids.png"));

        public override GH_Exposure Exposure => GH_Exposure.primary;

        // Generate your own — unique per component, permanent once shipped.
        public override Guid ComponentGuid => new Guid("e1f4a9b2-7c38-4d50-9a6f-3b4c5d6e7f80");
    }
}
