using System;
using System.Collections.Generic;

using Rhino.Geometry;

using Grasshopper.Kernel;

namespace MyPlugin.Components
{
    // -----------------------------------------------------------------------
    //  SunFilter — filter parallel points/values lists by a value range,
    //  then linearly rescale the surviving values so the kept set spans
    //  exactly [-0.5, +0.5]. Points pass through unchanged.
    // -----------------------------------------------------------------------
    public class SunFilterComponent : GH_Component
    {
        public SunFilterComponent()
          : base("Sun Filter", "SunFilter",
                 "Filters parallel points/values lists by a value range, then rescales the " +
                 "surviving values linearly so the kept set spans exactly [-0.5, +0.5]. " +
                 "Points pass through unchanged.",
                 "Geoboid", "Geoboid")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Points (e.g. from Sunlight Point Cloud), passed through unchanged.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Values", "V", "Values parallel to Points.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Min", "Min", "Lower filter bound, inclusive. Flipped bounds are allowed.", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Max", "Max", "Upper filter bound, inclusive. Flipped bounds are allowed.", GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "P", "Surviving points, original coordinates preserved.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Values", "V", "Surviving values, scaled so min -> -0.5 and max -> +0.5.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var pointsIn = new List<Point3d>();
            var valuesIn = new List<double>();
            double minVal = 0.0;
            double maxVal = 1.0;

            if (!DA.GetDataList(0, pointsIn)) return;
            if (!DA.GetDataList(1, valuesIn)) return;
            DA.GetData(2, ref minVal);
            DA.GetData(3, ref maxVal);

            // Script assumed equal-length parallel lists; guard it so a mismatch
            // warns instead of throwing an index exception.
            int count = pointsIn.Count;
            if (valuesIn.Count != count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    "Points and Values have different lengths; using the shorter of the two.");
                count = Math.Min(count, valuesIn.Count);
            }

            double lo = Math.Min(minVal, maxVal); // allow flipped bounds
            double hi = Math.Max(minVal, maxVal);

            var keptPts = new List<Point3d>();
            var keptVal = new List<double>();
            double obsMin = double.MaxValue;
            double obsMax = double.MinValue;

            for (int i = 0; i < count; i++)
            {
                double v = valuesIn[i];
                if (v < lo || v > hi) continue;

                keptPts.Add(pointsIn[i]);
                keptVal.Add(v);
                if (v < obsMin) obsMin = v;
                if (v > obsMax) obsMax = v;
            }

            // Rescale kept values to span [-0.5, +0.5].
            double obsRange = obsMax - obsMin;
            var scaled = new List<double>(keptVal.Count);

            if (obsRange < 1e-12)
            {
                for (int i = 0; i < keptVal.Count; i++) scaled.Add(0.0);
            }
            else
            {
                for (int i = 0; i < keptVal.Count; i++)
                {
                    double t = (keptVal[i] - obsMin) / obsRange; // 0..1
                    scaled.Add(t - 0.5);                          // -> -0.5..+0.5
                }
            }

            DA.SetDataList(0, keptPts);
            DA.SetDataList(1, scaled);
        }

        protected override System.Drawing.Bitmap Icon =>
            new System.Drawing.Bitmap(GetType().Assembly.GetManifestResourceStream("MyPlugin.Icons.SunFilter.png"));

        public override GH_Exposure Exposure => GH_Exposure.primary;

        // Generate your own — unique per component, permanent once shipped.
        public override Guid ComponentGuid => new Guid("c4d2e8f1-3a76-4b09-8e5c-1f2a3b4c5d6e");
    }
}
