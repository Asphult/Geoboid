using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Linq;

using Rhino;
using Rhino.Geometry;

using Grasshopper.Kernel;

namespace MyPlugin.Components
{
    public class SunlightCloudComponent : GH_Component
    {
        // base("full name", "nickname", "description", "ribbon tab", "ribbon panel")
        public SunlightCloudComponent()
          : base("Sunlight Point Cloud", "SunCloud",
                 "Reads a value,x,y,z CSV, optionally transforms the points, and builds a " +
                 "colour-mapped point cloud (blue = low, red = high).",
                 "Geoboid", "Geoboid")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            // The last argument is the default, so the component solves even with nothing wired in.
            pManager.AddTextParameter("CSV Path", "Path", "Full path to the CSV file. Columns: value, x, y, z.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Filter Zero", "FZ", "Skip rows whose value is exactly 0.", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Auto Range", "Auto", "Derive the colour domain from the data min/max.", GH_ParamAccess.item, true);
            pManager.AddNumberParameter("Domain Min", "Min", "Lower bound of the colour domain (used when Auto Range is false).", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Domain Max", "Max", "Upper bound of the colour domain (used when Auto Range is false).", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Gamma", "G", "Gamma applied to the normalised value before colouring.", GH_ParamAccess.item, 1.0);
            pManager.AddVectorParameter("Move", "Mv", "Translation applied after rotation.", GH_ParamAccess.item, Vector3d.Zero);
            pManager.AddNumberParameter("Rotation", "Rot", "Rotation in degrees about the world Z axis at the origin.", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Cloud", "C", "Colour-mapped point cloud.", GH_ParamAccess.item);
            pManager.AddPointParameter("Points", "P", "Transformed points.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Values", "V", "Source values, parallel to Points.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Locals seeded with the same defaults declared above.
            string csvPath = string.Empty;
            bool filterZero = false;
            bool autoRange = true;
            double domainMin = 0.0;
            double domainMax = 1.0;
            double gamma = 1.0;
            Vector3d moveVec = Vector3d.Zero;
            double rotAngle = 0.0;

            DA.GetData(0, ref csvPath);
            DA.GetData(1, ref filterZero);
            DA.GetData(2, ref autoRange);
            DA.GetData(3, ref domainMin);
            DA.GetData(4, ref domainMax);
            DA.GetData(5, ref gamma);
            DA.GetData(6, ref moveVec);
            DA.GetData(7, ref rotAngle);

            if (string.IsNullOrEmpty(csvPath) || !File.Exists(csvPath))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Invalid CSV path.");
                return;
            }

            if (gamma <= 0) gamma = 1.0;

            var ptList = new List<Point3d>();
            var valList = new List<double>();
            var ci = CultureInfo.InvariantCulture;

            string[] lines = File.ReadAllLines(csvPath);
            foreach (string raw in lines)
            {
                if (string.IsNullOrWhiteSpace(raw)) continue;
                string[] parts = raw.Split(',');
                if (parts.Length < 4) continue;

                if (!double.TryParse(parts[0].Trim(), NumberStyles.Float, ci, out double h)) continue;
                if (!double.TryParse(parts[1].Trim(), NumberStyles.Float, ci, out double x)) continue;
                if (!double.TryParse(parts[2].Trim(), NumberStyles.Float, ci, out double y)) continue;
                if (!double.TryParse(parts[3].Trim(), NumberStyles.Float, ci, out double z)) continue;

                if (filterZero && h == 0.0) continue;

                ptList.Add(new Point3d(x, y, z));
                valList.Add(h);
            }

            if (ptList.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No valid rows were parsed.");
                return;
            }

            // Rotation about world Z at origin, then translation.
            Transform xform = Transform.Identity;
            if (Math.Abs(rotAngle) > 1e-12)
                xform = Transform.Rotation(RhinoMath.ToRadians(rotAngle), Vector3d.ZAxis, Point3d.Origin);

            if (moveVec.Length > 1e-12)
                xform = Transform.Translation(moveVec) * xform;

            if (!xform.Equals(Transform.Identity))
                for (int i = 0; i < ptList.Count; i++)
                    ptList[i] = xform * ptList[i];

            // Colour domain.
            double vMin = autoRange ? valList.Min() : Math.Min(domainMin, domainMax);
            double vMax = autoRange ? valList.Max() : Math.Max(domainMin, domainMax);
            double vRange = Math.Max(vMax - vMin, 1e-9);

            var pc = new PointCloud();
            for (int i = 0; i < ptList.Count; i++)
            {
                double t = Math.Max(0, Math.Min(1, (valList[i] - vMin) / vRange));
                Color c = ValueToColor(Math.Pow(t, gamma));
                pc.Add(ptList[i], c);
            }

            DA.SetData(0, pc);
            DA.SetDataList(1, ptList);
            DA.SetDataList(2, valList);
        }

        // ---- colour helpers (unchanged from the original script) ----

        private static Color ValueToColor(double t) // blue low, red high
        {
            if (t < 0) t = 0;
            if (t > 1) t = 1;
            double hue = (1.0 - t) * 240.0;
            return ColorFromHSV(hue, 1.0, 1.0);
        }

        private static Color ColorFromHSV(double h, double s, double v)
        {
            h = ((h % 360) + 360) % 360; // normalise to 360
            int hi = (int)Math.Floor(h / 60) % 6;
            double f = h / 60 - Math.Floor(h / 60);

            double p = v * (1 - s);
            double q = v * (1 - f * s);
            double t = v * (1 - (1 - f) * s);

            double[][] sectors = new double[][]
            {
                new[] { v, t, p }, // hi = 0..5
                new[] { q, v, p },
                new[] { p, v, t },
                new[] { p, q, v },
                new[] { t, p, v },
                new[] { v, p, q }
            };

            double[] rgb = sectors[hi];

            return Color.FromArgb(
                255,
                (int)Math.Round(rgb[0] * 255),
                (int)Math.Round(rgb[1] * 255),
                (int)Math.Round(rgb[2] * 255));
        }

        protected override Bitmap Icon =>
            new Bitmap(GetType().Assembly.GetManifestResourceStream("MyPlugin.Icons.SunCloud.png"));

        // Where it sits in the ribbon panel.
        public override GH_Exposure Exposure => GH_Exposure.primary;

        // MUST be unique per component and MUST never change once shipped,
        // or saved definitions lose their wires on reopen. Generate your own.
        public override Guid ComponentGuid => new Guid("a7e3d1c4-6b29-4f08-9c5a-2d3e4f5a6b7c");
    }
}
