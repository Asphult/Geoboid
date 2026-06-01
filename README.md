# Geoboid

A set of computational-design components for **Grasshopper** (Rhino 8), built around a Craig Reynold's (1986) Boid Theory of Life. Utilises an environmental fields such as sunlight and airflow to drive generative geometry. It bundles a sun-data point-cloud pipeline, a 3D flocking solver with CFD wind and attractor fields, and a sun-modulated curve-to-mesh metaballer.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Rhino](https://img.shields.io/badge/Rhino-8%2B-brightgreen.svg)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20macOS-lightgrey.svg)
![.NET](https://img.shields.io/badge/.NET-7.0-512BD4.svg)

---

## Contents

- [What's inside](#whats-inside)
- [The pipeline](#the-pipeline)
- [Components](#components)
- [Installation](#installation)
- [Building from source](#building-from-source)
- [Packaging & publishing](#packaging--publishing)
- [Usage notes](#usage-notes)
- [Repository layout](#repository-layout)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## What's inside

| Component | Ribbon (tab → panel) | Purpose |
|---|---|---|
| **Sunlight Point Cloud** | Geoboid → Analysis | Reads a `value,x,y,z` CSV and builds a colour-mapped point cloud, with optional transform. |
| **Sun Filter** | Geoboid → Analysis | Filters parallel point/value lists by a range, then normalises to `[-0.5, +0.5]`. |
| **Boids** | Geoboid → Simulation | 3D flocking with attractor field, obstacles, teleport portals and optional CFD wind. |
| **Curve Metaballer** | Geoboid → Mesh | Volumises curves into a mesh via a smooth-min SDF, modulated by a sampled sun field. |

---

## The pipeline

The components are designed to chain. A typical graph:

```
sunlight CSV ──▶ Sunlight Point Cloud ──▶ Sun Filter ─┐
                                                       ├──▶ Boids (attractor field) ──▶ trails / mesh inputs
CFD vectors ───────────────────────────────────────────┘   (wind field)

curves ─────────▶ Curve Metaballer ◀── sun points / values
```

The sun data is the common thread: the same point/value pair that colours the cloud can steer the flock (as an attractor/detractor field) and modulate the metaballer's tube thickness. Each component is also fully usable on its own.

---

## Components

### Sunlight Point Cloud
Parses a CSV of `value, x, y, z` rows (e.g. annual sunlight-hours per point), optionally rotates about world Z and translates the result, then maps each value to a colour.

- **Inputs:** CSV path, filter-zero toggle, auto-range toggle, manual domain min/max, gamma, move vector, rotation (degrees).
- **Outputs:** coloured `PointCloud`, transformed points, source values.

### Sun Filter
Keeps only the points whose value falls inside `[Min, Max]` (flipped bounds allowed), then linearly remaps the surviving values so the kept set spans exactly `[-0.5, +0.5]`. Points pass through unchanged.

- **Inputs:** points, values, min, max.
- **Outputs:** surviving points, rescaled values.

### Boids
A frame-stepped flocking solver. Alignment / cohesion / separation, plus an attractor-point field, obstacle bouncing, curve-to-curve teleport portals, a closed bounding volume (bounce or wrap), and optional trilinearly-sampled CFD wind. Outputs a magnitude-coloured cloud of the CFD field for reference.

- **Inputs:** reset, wrap, count, speed, trail length, neighbourhood/cohesion/alignment/separation weights, separation distance, attractor points + values + weight, bounding volume, boid geometry, obstacles, teleport A/B curves, CFD points + vectors + transform, wind factor.
- **Outputs:** trail curves, locations, directions, oriented geometry, CFD magnitude cloud.
- **Drive it with a Grasshopper Timer.** It advances one frame per solve; toggle **Reset** to re-seed.

### Curve Metaballer
Coats each curve in a string of overlapping spheres, blends them into one continuous signed-distance field with a smooth-min, and extracts a watertight triangle mesh with marching cubes. Sphere radius and blend are modulated per-point by a sampled sun value (IDW over a spatial hash), with optional radius noise and end-taper.

- **Inputs:** curves, sun points + values, sun search radius, radius, radius factor, blend, blend factor, voxel size, noise amp + scale, taper, smooth passes.
- **Outputs:** `Mesh`, coloured debug sample cloud.
- **Voxel Size is the main quality/speed knob** smaller is finer but much slower and heavier.

---

## Installation

### From Rhino's Package Manager (recommended)
1. In Rhino 8, run the `PackageManager` command.
2. Search for the plugin by name and click **Install**.
3. Restart Rhino. The components appear under the **Geoboid** tab in Grasshopper.

### Manual
1. Download the latest `.gha` from the [Releases](../../releases) page.
2. Copy it to your Grasshopper libraries folder:
   - Windows: `%AppData%\Roaming\Grasshopper\Libraries`
   - macOS: `~/Library/Application Support/McNeel/Rhinoceros/packages/8.0/Grasshopper`
3. Right-click the `.gha` → **Properties** → **Unblock** (Windows only).
4. Restart Rhino and Grasshopper.

**Requirements:** Rhino 8 (Windows or macOS). The components are cross-platform.

---

## Building from source

**Prerequisites:** the [.NET SDK](https://dotnet.microsoft.com/download), Rhino 8, and Visual Studio 2022 or VS Code. The `RhinoCommon` and `Grasshopper` libraries are pulled in as NuGet packages  no manual DLL references needed.

```bash
git clone https://github.com/<you>/Geoboid.git
cd Geoboid
dotnet build -c Release
```

The compiled `Geoboid.gha` lands in `bin/Release/`. For an iterative debug loop, the Rhino plugin templates configure F5 to launch Rhino with the plugin loaded. If you don't have the templates yet:

```bash
dotnet new install Rhino.Templates
dotnet new grasshopper --name Geoboid
```

Then drop the four `.cs` files from `src/` into the generated project, each component self-registers via its `GH_Component` subclass, and the template's `GH_AssemblyInfo` class supplies the plugin metadata.

---

## Packaging & publishing

Distribution uses [Yak](https://developer.rhino3d.com/guides/yak/), Rhino's package manager. From a folder containing the built `.gha`, an `icon.png`, and any `misc/` docs:

```bash
# locate yak Windows: "C:\Program Files\Rhino 8\System\yak.exe"
#               macOS:   /Applications/Rhino 8.app/Contents/Resources/bin/yak

yak spec          # generate manifest.yml from the .gha
# edit manifest.yml (name, version, authors, description, keywords, icon)
yak build         # produces e.g. Geoboid-0.1.0-rh8-win.yak
yak login         # requires a Rhino Account
yak push Geoboid-0.1.0-rh8-win.yak
```

A starter `manifest.yml` lives in this repo. Please note that package versions are immutable once pushed, bump the version for every release. Push to the test server first (`yak push --source <test-server-url>`) if you want a dry run.

---

## Usage notes

- **Boids needs a Timer.** Without a recompute trigger it won't animate. It also steps on *any* recompute, so avoid wiring volatile upstream inputs you don't want advancing frames.
- **Boids scales as O(n²)** in the compiled component. It's smooth into the low hundreds; for larger flocks use the spatial-hash script variant in [`scripts/`](#repository-layout), which keeps behaviour identical while cutting the neighbour search to roughly O(n·k). Current Rhino PackageManager v1.0.2 does not incorporate this.
- **Metaballer Voxel Size** dominates both mesh quality and solve time. Start coarse and refine. With `Noise Amp` above 0, keep `Noise Scale` above 0 too (a zero scale produces NaN radii).
- Parallel-list inputs (points/values, sun points/values) should be equal length; mismatches warn and degrade gracefully rather than throwing.

---

## Repository layout

```
Geoboid/
├── src/
│   ├── SunlightCloudComponent.cs
│   ├── SunFilterComponent.cs
│   ├── BoidsComponent.cs
│   └── CurveMetaballerComponent.cs
├── scripts/
│   └── boids_spatial_hash.cs      # script-component variant, spatial-hash optimised
├── manifest.yml                   # Yak package manifest
├── icon.png                       # package icon (24×24 component icons TBD)
├── README.md
└── LICENSE
```

---

## Contributing

Issues and pull requests are welcome, please note below:

- Keep the simulation/geometry math in plain helper methods so it stays testable outside Rhino.
- One component per file; give every new component a freshly generated `ComponentGuid` and never change it after release.
- Match the existing `Tab → Panel` categorisation, or open an issue to discuss a new one.

---

## License

Released under the [MIT License](LICENSE). You're free to use, modify, and distribute this, including commercially, provided the copyright notice is retained.

---

## Acknowledgements

- Built on [RhinoCommon](https://developer.rhino3d.com/) and the Grasshopper SDK by Robert McNeel & Associates.
- The marching-cubes edge and triangle lookup tables are the standard ones widely circulated from Paul Bourke's reference implementation.
- Flocking follows the classic boids model (alignment, cohesion, separation) introduced by Craig Reynolds.
