# ModelMate

## Credits: projects, libraries and papers this builds on

**Code ported into this project**
- **[Instant Meshes](https://github.com/wjakob/instant-meshes)** (BSD 3-Clause) by Wenzel Jakob, Marco Tarini, Daniele Panozzo and Olga Sorkine-Hornung: *Instant Field-Aligned Meshes*, ACM Transactions on Graphics (SIGGRAPH Asia) 2015. The quad remesher in `src/Core/Quad/` (`FieldMath`, `Hierarchy`, `FieldSolver`, `Extraction`) is a C# port of its field optimization and mesh extraction. The full license is in [`THIRD_PARTY.md`](THIRD_PARTY.md).

**Libraries used (through Godot, not vendored here)**
- **[Godot Engine](https://godotengine.org) 4.7** (MIT): the runtime, UI, rendering, glTF import and export (`GLTFDocument`), `Skeleton3D`/`Skin`, and `SkeletonProfileHumanoid` (the bone names and hierarchy of the humanoid rig).
- **[.NET 8](https://dotnet.microsoft.com) and the Godot .NET SDK** (MIT): the C# toolchain.
- **[xatlas](https://github.com/jpcy/xatlas)** (MIT): optional UV unwrapping (`--unwrap xatlas`), called through Godot's `ArrayMesh.LightmapUnwrap`. Godot compiles it into the editor only. The default everywhere is the built-in unwrapper (`ChartUnwrap.cs`, `src/Core/Uv/`).

**Algorithms implemented from publications (no code copied)**
- **Surface Nets**: S. Gibson, *Constrained Elastic Surface Nets*, MICCAI 1998. Used in the voxel remesher (`SurfaceNets.cs`).
- **Quadric error metrics**: M. Garland and P. Heckbert, *Surface Simplification Using Quadric Error Metrics*, SIGGRAPH 1997. Used for decimation (`QemDecimator.cs`).
- **Normal voting tensor** (sharp edges and corners at a chosen scale): D. L. Page, Y. Sun, A. F. Koschan, J. Paik and M. A. Abidi, *Normal Vector Voting: Crease Detection and Curvature Estimation on Large, Noisy Meshes*, Graphical Models 2002. Used by the hard-surface mode (`Quad/Features.cs`); the edges then constrain the Instant Meshes fields, as its own crease handling does.
- **Geodesic voxel skinning**: the idea of measuring bone distance through the voxelized interior comes from O. Dionne and M. de Lasa, *Geodesic Voxel Binding for Production Character Meshes*, SCA 2013. `VoxelGeodesicWeights.cs` is a simplified variant (Dijkstra over interior voxels with an inverse-distance falloff), not their full method.
- **Closest point on a triangle**: C. Ericson, *Real-Time Collision Detection*, 2004 (`Geo.ClosestPtTri`).
- **Rotating calipers** (minimum-area bounding rectangle): G. Toussaint, *Solving Geometric Problems with the Rotating Calipers*, 1983. **Skyline bin packing**: J. Jylänki, *A Thousand Ways to Pack the Bin*, 2010. Both are used by the chart unwrapper (`ChartUnwrap.cs`, `Uv/AtlasPack.cs`, which packs by each chart's outline rather than its rectangle).
- **Least squares conformal maps**: B. Lévy, S. Petitjean, N. Ray and J. Maillot, *Least Squares Conformal Maps for Automatic Texture Atlas Generation*, SIGGRAPH 2002. **As-rigid-as-possible parameterization**: L. Liu, L. Zhang, Y. Xu, C. Gotsman and S. Gortler, *A Local/Global Approach to Mesh Parameterization*, SGP 2008. Both flatten charts in `Uv/ChartFlatten.cs`.
- **Maximum flow**: E. A. Dinic, *Algorithm for Solution of a Problem of Maximum Flow in Networks with Power Estimation*, 1970. The minimum cut places seams between neighbouring charts (`Uv/MaxFlow.cs`, `Uv/ChartMerge.cs`).
- **Ray–triangle intersection**: T. Möller and B. Trumbore, *Fast, Minimum Storage Ray/Triangle Intersection*, 1997 (`Geo.RayTri`).

**Icons**
- **[Lucide](https://lucide.dev)** (ISC): the toolbar and panel icons, embedded in `src/App/Icons.cs`. The license is in [`THIRD_PARTY.md`](THIRD_PARTY.md).

**Tools used only during development (not part of the app)**
- **[Pillow](https://python-pillow.org) and [NumPy](https://numpy.org)** (Python): compared rendered screenshots during development.
- **curl**: downloaded the Instant Meshes reference source for porting.

---

A small Godot 4.7 (C#) tool. It turns an unrigged static glTF (a dense sculpt or AI-generated mesh) into a rigged, UV-unwrapped `.glb`, ready to texture in a PBR painter:

**import → fit skeleton (humanoid, creature or your own) → retopologize (quad or voxel) → auto skin weights → UV unwrap → export**

ModelMate works on geometry only. The imported file's materials, textures, vertex paint, lights and cameras are ignored, and the export carries the mesh, UVs, skeleton and weights with a plain untextured material. The viewport shows models like Blender's default cube: grey matte under one light, with the wireframe on top (toggle *Wireframe*). On a dense sculpt the lines fade out until you zoom in.

Everything runs inside Godot, with no Blender or other external tools. With the humanoid rig, bone names and parent bones match Godot's `SkeletonProfileHumanoid`, which is what humanoid retargeting expects. To retarget, open the `.glb` in your game project's import dialog and set Skeleton3D → Retarget → Bone Map → `SkeletonProfileHumanoid`.

Speed on an M-series Mac, quad mode, at the default budget of 8,000 triangles: about 3–5 s for a 46k-triangle model and about 8 s for a 3.8M-triangle one.

## Running

```sh
G=/Applications/Godot.app/Contents/MacOS/Godot
dotnet build                     # or let Godot build on launch
$G --path .                      # UI
```

**UI workflow**:
1. Before anything is loaded the window shows one button, **Import Mesh** (tick *Model faces −Z* first if it does). Afterwards, the toolbar in the top-left corner has Import, Retopology, Export, the **Show** menu (source, retopologized mesh, wireframe, skeleton), **Settings** (faces −Z, UV padding for the texture size you will paint at) and the Console. The top right of the view has **Frame** and an orientation cube: drag it to turn the view, or tap a face to view from that side (Front is +Z, Left is the character's left, +X). A dimmed overlay names the step while one runs.
2. **Retopology** opens by itself after every import (drag it by its title; the toolbar reopens it). Pick a mode and a **target triangle count**, then **Retopologize**. Quad mode is the default. In voxel mode the voxel resolution sets the detail and the result is decimated to the target. Set **Surface** to *Hard surface* for buildings, gadgets and props: sharp edges and corners are kept, edge loops run along them, and they shade sharp (see *Hard surface* below).
3. **Export** works from here on. Without a skeleton the export is a static mesh with UVs.
4. Optional, in the side panel (each section collapses, and remembers it): **Skeleton & weights**. Pick a rig preset and click **Fit skeleton**, then left-drag joints to adjust them (left/right edits mirror by default). Edit the skeleton with **Add child**, **Delete**, **Rename**, **Parent**, **Deforms mesh** and **Symmetrize** (copies every `Left*` bone onto its `Right*` partner). **Save** stores it as a preset you can reuse on other models; **Load preset…** fits a preset `.json` from anywhere. **Compute weights** (Export does it too when a skeleton is fitted), and tick *Show selected bone's weights* for a heatmap, choosing the bone in the list or by clicking a joint.
5. Optional: **Preview pose** with the rotation sliders or **Test pose** (humanoid: a standard bend; rigs from a preset with named poses, such as `blob_biped`: each press shows the next pose). Use **Reset pose** before editing joints again.
6. **Export** unwraps the UVs, writes the file, then re-imports it and runs the checks listed below. The results appear in the Console, which opens by itself if a check fails.

Camera: left-drag on empty space or right-drag to orbit, Shift + drag to pan, mouse wheel or trackpad (pinch or scroll) to zoom. **Frame model** re-centres the view.

**Headless CLI** (the same pipeline, plus verification; exits 0 only if every check passes):

```sh
$G --headless --path . -- --in model.glb --out model_rigged.glb \
   [--retopo quad|voxel] [--tris 8000] [--sizing auto|uniform|adaptive] [--detail 256] [--flow 0] \
   [--no-fit] [--no-pin] [--dump-quads layout.obj] [--seed 1] \
   [--voxel 256] [--close 1] [--tex 1024] [--relax 4] [--falloff 3] \
   [--rig humanoid|single|blob_biped|preset.json|none] [--save-rig out.json] \
   [--rotate-y 180] [--no-joint-aware] [--probe-weights] [--unwrap charts|cone|xatlas] [--dump-uv uv.png] [--uv-debug] \
   [--surface organic|hard] [--crease 30] [--dump-features edges.obj] [--dump-crease-misses misses.obj]
$G --headless --path . -- --in model.glb --rig humanoid --deform-report <dir> [--budgets 1000,2000] [--modes quad,quadflow,voxel]
$G --headless --path . -- --selftest-fields                      # quad remesher + rig editing self-tests
$G --headless --path . -- --make-test-asset res://test [--dense] # procedural test humanoids (T-pose, open, A-pose, ~3.8M-tri dense)
./test/regress.sh                                                # runs every test model through the CLI, both modes
./test/sweep.sh                                                  # quality table across budgets (1k–16k triangles)
$G --path . -- --render out.glb shot.png [--views front,side,back,top,bottom,three] [--wire] \
   [--pose | --pose-preset rigs/blob_biped.json waddle_left]     # snapshot (windowed); also accepts a quad .obj
$G --path . -- --ui-smoke res://test/humanoid.glb <dir>          # drives the UI end to end
$G --path . -- --touch-ui                                        # the iPad (touch) UI on the desktop
$G --path . -- --touch-test                                      # synthetic touch gestures: orbit, joint drag, pinch
./test/ios_build.sh [--sim [--smoke]]                            # iOS Xcode project; iPad simulator build and smoke test
```

## Running on an iPad

ModelMate runs on an iPad as an ordinary app. It has been tested on an iPad Pro 12.9" (5th generation, M1), where the whole UI flow (the smoke test below) passes. Measured there:

| model | budget | retopology | weights | export | peak memory |
|---|---|---|---|---|---|
| Lil gug (creature, 254k-triangle source) | 2k triangles | 9.1 s | 0.7 s | 1.7 s | 1.0 GB |
| test humanoid (46k-triangle source) | 8k triangles | 1.5 s | 0.2 s | 1.0 s | 0.7 GB |

iOS lets ModelMate use about 5.1 GB on that iPad; the log reports memory after each step, including how much more iOS allows.

The UI scales for touch:

| gesture | does |
|---|---|
| one finger on empty space | orbit |
| one finger on a joint | move the joint |
| two fingers | pinch to zoom, drag to pan |
| **Frame model** button | re-centre the camera |

**Getting models in.** Share a `.glb` from another app (for example Nomad Sculpt's export) and pick **ModelMate** in the share sheet: the model is copied into `Models/` and imported. You can also put `.glb`/`.gltf` files in the app's folder in the Files app (**On My iPad › ModelMate › Models**); **Import** opens that folder.

**Sending models out.** **Export & share…** writes `Exports/<model>_rigged.glb` and then opens the share sheet, so you can send it to AirDrop, Files, iCloud Drive or any app that accepts glTF. The file stays in `Exports/` either way. Saved rig presets go to `rigs/`.

ModelMate declares the glTF types the same way as other iPad apps (`org.khronos.glb` for `.glb`, `org.khronos.gltf` for `.gltf`, plus `org.khronos.gltf.binary` and `public.geometry-definition-format`) so iOS offers it for those files. Other 3D formats shared to it are refused with a message. Both directions go through the Objective-C runtime from C# (`src/App/IosBridge.cs`): a small class registered with Godot's application delegate receives shared files, and the share sheet is a `UIActivityViewController`. A file shared while ModelMate isn't running arrives before Godot loads the C# code, so a tiny native plugin (`ios/plugins/MMLaunchURL`, built by `test/ios_build.sh`) catches it at launch and leaves it in `tmp/Received` for the app to import.

**Building and installing** (needs a Mac with Xcode, the Godot 4.7.2 .NET export templates, and a free or paid Apple ID):

1. `./test/ios_build.sh` exports the Xcode project to `build/ios/ModelMate.xcodeproj`. (With the iPad connected, `TEAM_ID=<your team ID> ./test/ios_build.sh --device` does steps 1–4 from the command line, and `--device --smoke` then runs the UI smoke test on the iPad and copies its screenshots and log back to `build/ios/`. `SMOKE_MODEL=lil_gug.glb SMOKE_RIG=lil_gug SMOKE_TRIS=2000` picks another case.)
2. On the iPad, turn on **Settings › Privacy & Security › Developer Mode**, then connect it with a cable.
3. Open the Xcode project. Under **Signing & Capabilities**, choose your Personal Team. If Xcode says the bundle ID is taken, change `dev.warent.modelmate` to something unique.
4. Pick the iPad as the run destination and press **Run**. On the first launch, trust the developer under **Settings › General › VPN & Device Management**.

With a free Apple ID, the app stops opening after 7 days; run it from Xcode again to renew it.

**Testing in the simulator.** `./test/ios_build.sh --sim` builds for the iPad simulator, installs the app and copies the test models into its `Models/` folder. `--sim --smoke` also runs the UI smoke test inside the simulator. Godot ships its simulator library for x86_64 only, and iOS 26 simulators no longer run x86_64 apps on Apple silicon, so this needs an iOS 18 (or older) simulator runtime (`SIM="<device name>"` picks it). The simulator uses the Mac's CPU and memory, so its timings say nothing about the device.

**Debugging on the device.** Debug builds write the Godot log to `logs/godot.log` in the app's folder and read commands from `cmd.txt` there (one per line: `shot NAME` saves a screenshot to `shots/NAME.png`; `ui-smoke IN OUT` runs the smoke test). With `xcrun devicectl device copy to/from … --domain-type appDataContainer --domain-identifier dev.warent.modelmate`, a connected Mac can drive and inspect the running app. Godot also keeps its `shader_cache` and `vulkan` folders there; they are caches and safe to ignore.

**What differs from the desktop.** Nothing in the pipeline: UV unwrapping uses the same built-in unwrapper on both (Godot's xatlas module isn't in exported apps, and is no longer the default on the desktop either). The log reports the time and memory of each step. Keep the app in front while it works: iPadOS pauses apps in the background.

## Rig presets
| Preset | For | Bones |
|---|---|---|
| `humanoid` | T- or A-pose characters; names and hierarchy match Godot's `SkeletonProfileHumanoid` | 23 (no fingers, eyes or jaw) |
| `blob_biped` | Round creatures (Kirby-like): body, stubby arms, feet at the bottom, horns | 9; delete the horns if there are none, add a tail with **Add child** |
| `single` | Props | 1 |

JSON presets (`rigs/*.json` built in, user presets in Godot's `user://rigs`) store joints as fractions of the model's bounding box, so a preset refits to any model of that kind. Built-in presets re-center joints inside the mesh after fitting; saved presets reload your joints exactly (`"autoCenter": false`). A preset can also carry named test poses (`"poses"`), used by `--pose-preset` and the deformation report. `test/rigs/lil_gug.json` is an example fitted to a real creature.

## What the input should look like
- One character, upright: +Y up, **facing +Z**, character's left on +X. This is the glTF/Godot convention. If a model faces -Z, tick *Model faces -Z* before importing, or pass `--rotate-y 180`. Otherwise left and right get swapped, and the checks **can't detect this**, because they only compare the rig with itself.
- For the humanoid rig: T-pose or A-pose. The arm angle is detected from the bounding box.
- Use `--rig blob_biped` for round creatures and `--rig single` for props, or build your own skeleton in the UI and save it as a preset.
- Only geometry is read. If the file has no normals (Nomad exports positions only), they are computed.

## How it works

| Stage | Method (`src/Core`) |
|---|---|
| Inside test | `VoxelGrid`: a narrow-band distance field. A point counts as inside if either test says so: a flood fill that first seals gaps (dilate, fill, erode), or a ray-crossing vote along X, Y and Z. This handles holes, overlapping parts and inverted normals. |
| Base mesh | `SurfaceNets`, then `MeshCleanup` (splits non-manifold edges and vertices), then `Relax` (smoothing along the surface plus guarded snapping to the source). This closed, evenly dense surface feeds both remesh modes and the weights. |
| Hard surface (`--surface hard`) | `Quad/Features.cs` finds sharp edges and corners on the base mesh; quad remeshing aligns to them, puts vertices exactly on them and keeps them there; hard edges split normals and attract UV seams. See *Hard surface* below. |
| Quad remesh (default) | `Quad/`: Instant Meshes port. A multiresolution hierarchy, a 4-RoSy orientation field and a 4-PoSy position field solved coarse to fine, then graph and face extraction, triangulation along each quad's shorter diagonal, hole filling and quad-preserving reprojection. If the result isn't closed and manifold or is under 70% quads, it falls back to voxel mode and the export checks fail, so a fallback is never silent. |
| Voxel remesh | The base mesh decimated by `QemDecimator`: quadric-error edge collapse with manifold and flip guards, up to 6× more detail kept near joints, and an edge-length cap. |
| Skeleton | `RigPreset` fits a preset to the bounding box (`HumanoidTemplate` for the humanoid), then centers joints inside the mesh by casting rays through it. `RigEdit` does the structural edits and keeps the skeleton valid (one root, parents before children, unique names, no cycles). Each bone's +Y points at its child. |
| Weights | `VoxelGeodesicWeights`: distance from each bone is measured *through the mesh interior* (Dijkstra over voxels), minus the bone's local thickness. Weight = 1/d^falloff. They're solved once on the dense base surface, smoothed over a world-space radius (2% of model height), mirror-averaged when the model and rig are both symmetric (`Symmetry`), cut to the top 4 bones and normalized, then transferred to the output mesh by closest point. So weights don't change with the triangle budget or the remesh, and they're identical across UV seams. |
| UVs | `ChartUnwrap` (see *UV unwrapping* below): few large charts, with seams placed where a painter's camera sees them least. Charts are padded for the chosen texture size (8 texels at 1024) so a painter's bleeding doesn't cross islands. |
| Export | `RigBuilder` and `Exporter` (`GltfDocument`), then `Verify.RoundTrip`. |

## UV unwrapping

Seams are what show in a PBR painter: a brush stroke or a fill across a seam breaks wherever the two sides' texels don't line up. The unwrapper therefore aims for few charts, with seams where they show least, rather than for the tightest packing:

1. **Small starting charts.** Region growing over the triangles, a quad's two triangles always together, within a 50° normal cone, re-grown tighter where a planar projection would fold.
2. **Seam cost.** Each vertex is looked at from 26 directions around the model; a ray toward each must escape the mesh. Directions toward the front (+Z, the rig templates' convention) weigh most, the sides and top less, the back and underside least, and vertices skinned to a `Head` bone weigh extra. So a seam along an edge costs its length times how visible it is, and creases, armpits, inner legs, the back and the underside are cheap.
3. **Merging.** Adjacent charts are joined, the most visible shared seam first. A join is kept if the result flattens (LSCM, then ARAP; `ChartFlatten`) with no flipped triangles, no overlap, shape stretch (s1/s2) p95 ≤ 1.3 and area stretch p95 ≤ 1.45. A join that closes a ring (around a limb, or a chart that would wrap round the body) is first tried as an island with a hole; if that doesn't flatten, it is cut open along the cheapest path between its borders, and only kept if that cut costs well under the seam it removes.
4. **Seam placement.** For the most visible remaining seams, the two charts' triangles are re-split by a minimum cut between their deepest parts, so the seam moves to the cheapest route nearby (kept only if both charts still flatten and the cost drops).
5. **Packing.** Charts are dropped onto a skyline by their real outline (16 turns each), at one uniform texel density.

`--unwrap cone` stops after step 1 and packs planar projections by rectangle (the unwrapper before merging). `--dump-uv uv.png` writes the layout, `--uv-debug` prints every merge decision.

Measured against `--unwrap cone` (the unwrapper before merging):

| Model | Charts | Seam length (× model height) | Texture covered | Shape stretch p95 | Time |
|---|---|---|---|---|---|
| Lil gug, 8k triangles | 127 → 5 | 28 → 9.2 | 52% → 49% | 1.49 → 1.25 | 0.2 → 1.6 s |
| Lil gug, 2k triangles | 42 → 6 | 20 → 7.7 | 54% → 48% | 1.50 → 1.28 | 0.2 → 0.6 s |
| Humanoid test model, 8k | 110 → 14 | 20 → 8.6 | 49% → 55% | 1.33 → 1.22 | 0.2 → 1.5 s |

On Lil gug at 8k the front of the face has no seam (only a short loop inside the mouth); on the humanoid the torso and head seams sit on the back and the limb seams on their inner and back sides. Larger meshes take longer: about 7 s at 16k triangles (times on an M-series Mac).

## Hard surface

Organic remeshing smooths everything a little: edges get rounded, corners cut, and quads cross creases at any angle. *Hard surface* (`--surface hard`) keeps them:

1. **Sharp edges and corners.** For each base-mesh vertex, the normals of the faces within a few voxels (or a quarter quad) are summed into a tensor: one dominant direction means a plane, two an edge (it runs along the third), three a corner. Edges are thinned to the vertices where the turn peaks, and each edge point and corner is moved to where planes fitted to the *source* on each side meet, which restores the edge the sculpt and the voxel base mesh rounded off. A corner counts only where three planes meet in a point. Edges turn by more than the *sharp edge* angle (30° by default).
2. **Fields along them.** The orientation field runs along every edge; a lattice line lies on each edge (it may slide along it) and a lattice point on each corner.
3. **Vertices on them.** Extraction places each vertex that holds an edge or corner exactly on it; smoothing and the surface fit then leave corners where they are and keep edge vertices on their edge. A quad whose opposite corners both sit on an edge is split along it.
4. **Small features.** Where two parallel edges are closer than the spacing (a step, a ledge, a plate), the spacing there shrinks to their distance (no finer than 0.6 of the spacing), and the rest of the model coarsens to keep the budget. The curvature term is off: it would refine every edge.
5. **Shading and seams.** Edges sharper than the angle are hard: normals split there (in the export and the viewport), and UV seams cost a tenth there, so they tend to follow them.

Measured on three hard-surface test models (`--make-test-asset res://test --hard`: a remote, a house, a flashlight), against organic remeshing at the same budget:

| Model | Sharp-edge length along a quad edge, 2k | 8k | Corner to nearest vertex (median) |
|---|---|---|---|
| Remote | 38% → 78% | 61% → 84% | 0.2–0.3ρ → 0 |
| House | 35% → 76% | 62% → 78% | 0.2–0.3ρ → 0–0.05ρ |
| Flashlight | 32% → 71% | 40% → 51% | 0.2ρ → 0 |

Edges that are still missed are on features smaller than about two voxels or half a quad (the flashlight's grip rings stand out by 0.01 units). `--dump-features` writes the detected edges and corners as an OBJ, `--dump-crease-misses` the source's sharp-edge samples that no quad edge follows.

## Deformation report: quad vs voxel

`./test/deform_report.sh [outdir]` (or `--deform-report <dir>` on the CLI) poses every variant (mode × budget) with the *same* weights, transferred from a dense reference, so the numbers measure tessellation and edge layout only. It writes `report.md`, `report.csv` and posed OBJs per model, and renders the named poses at mobile budgets.

What it shows on the test models (Lil gug, the A-pose humanoid and the lumpy humanoid, 1k–16k triangles):

Average p99 error over the bone poses, % of model height, defaults for both modes (quad: auto spacing; both: surface fit):

| model | | 1k | 2k | 4k | 8k |
|---|---|---|---|---|---|
| A-pose humanoid | total posed (quad / voxel) | **0.62** / 0.68 | 0.33 / **0.32** | **0.23** / 0.24 | **0.16** / 0.21 |
| | bending | **0.26** / 0.36 | **0.16** / 0.24 | **0.11** / 0.23 | **0.07** / 0.21 |
| lumpy humanoid | total posed | 0.65 / 0.65 | 0.41 / **0.29** | 0.25 / **0.22** | 0.16 / 0.16 |
| | bending | **0.25** / 0.32 | **0.18** / 0.24 | **0.12** / 0.20 | **0.08** / 0.15 |
| Lil gug | total posed | 1.29 / **1.02** | 0.74 / 0.74 | **0.50** / 0.54 | **0.36** / 0.38 |
| | bending | **0.88** / 0.92 | **0.62** / 0.69 | **0.45** / 0.52 | **0.34** / 0.36 |

*Bending* is how much worse the mesh gets when posed; *total posed* is what a viewer sees in the pose (shape error plus bending).

- **Quad mode bends more cleanly at every budget.** Voxel mode's bending error stays high even at 8k because decimation leaves long slivers that span joints.
- **Overall they're close**: quad wins or ties in 8 of 12 cells. Voxel mode still matches shape better in a few (Lil gug at 1k, fine ridges at 2k–4k), because its long slivers follow ridges that roughly square quads can't.
- Quad stays the default because it's the mode that animates cleanly; voxel mode is there when a static silhouette matters more.

How quad mode got here, measured with the same report:
1. **Uniform spacing** was worse at matching shape at mobile budgets (e.g. 1.19 vs voxel 0.75 total on Lil gug at 2k), because triangles were spread by area, not by need.
2. **Adaptive spacing** (`--sizing auto`, below) spends more quads where the surface curves.
3. **Tip pinning** (`TipDetector`, on by default in quad mode, `--no-pin` to disable): sharp convex tips (horn tips, noses, box corners) get a lattice point pinned onto them through the position constraints of the field solver, so they aren't cut short. It removed the "feature shortened" warning on Lil gug's horns at 2k and improved most cells slightly (Lil gug at 1k: 1.36 → 1.29 total), at a cost of about 5% on the noisy lumpy humanoid at 1k–2k.
4. **Surface fitting** (`SurfaceFit`, on by default in both modes, `--no-fit` to disable) moves vertices slightly off the surface along their normals so the faces fit the dense surface in a least-squares sense. A coarse mesh with every vertex *on* a curved surface has every face cutting inside it; balancing that roughly halves shape error. It took Lil gug at 2k from 0.86 to 0.74 in quad mode and the A-pose humanoid at 1k from 1.01 to 0.68 in voxel mode.

(The report builds both modes from the same 256-cell base mesh, which is also the shipping default for both.)

**Adaptive spacing** (quad mode, default `--sizing auto`). A sizing field gives each point of the surface a target edge length from its curvature (spacing ∝ κ^-½, the rule for constant chord error), limited to a 6× range and to gradual changes (≤ 0.2 per unit distance), and rescaled so the total meets the triangle budget (one recalibration pass if extraction misses it by more than 4%). `auto` builds both the uniform and the adaptive remesh from the same base mesh and keeps the one closer to the surface (99th-percentile deviation), so shape fidelity never gets worse; it costs one or two extra remesh passes. Adaptive spacing puts its triangles where density changes, so the quad share drops (83–94% at 1k–4k, vs 93–97% uniform). A thickness term (`--adapt 0.5`) is available but measured as a wash next to curvature alone.

**Skeleton-guided edge flow** (`--flow 0.7`, quad mode) pulls the field toward each region's bone axis, with extra weight at joints. Measured with the same report at 1k–4k triangles it didn't earn a place as the default: it cut collapsed triangles on the A-pose humanoid at 1k (1.2 → 0.7 per 1,000) but made total posed error slightly worse on Lil gug at every budget (by up to 8%) and on the lumpy humanoid at 1k and 4k. On these models the unguided field already runs along the limbs, so guidance mostly adds singularities. It stays available as an experimental CLI option.

## Checks run on export
The mesh is manifold. UVs are in [0,1]. Weights sum to 1 with at most 4 bones per vertex. Seam vertices carry identical attributes. In voxel mode the triangle budget is met.

UV layout: no triangle is flipped against its chart. With the built-in unwrapper (the default), rasterizing the layout at the texture size also shows no texel claimed by two triangles, charts at least 2 texels apart, and 95th-percentile area stretch at most 1.6. For xatlas layouts (`--unwrap xatlas`, editor builds) those figures are reported as `[METRIC]` lines, since xatlas occasionally packs a few texels of overlap. Also reported: the number of charts, the total seam length (in model heights) and the shape stretch.

Hard surface (`--surface hard`): at least 45% of the source's sharp-edge length runs along a quad edge, the median distance from it to the remesh is at most 0.1ρ, and the median corner is within 0.1ρ of a vertex. The same figures are reported for organic remeshing as a `[METRIC]` line.

In quad mode:
- no fallback was used;
- at least 80% quads (catches broken extraction; uniform spacing typically gives ≥93%, adaptive 83–94%);
- no boundary edges, and the same topology (Euler characteristic) as the base mesh;
- the face count is within ±20% of the target;
- surface deviation from the base mesh: mean under 0.15 and 99.5th percentile under 0.5 of the quad spacing. A larger worst-case deviation (typically a thin tip cut short) is reported as a `[WARN]` with its location.

Also reported as `[METRIC]` lines, without failing the run: the share of valence-4 vertices, the worst surface deviation, and (humanoid rig) the number of sides around each limb.

After re-import, for any rig:
- the mesh has UVs;
- the rest pose reproduces the mesh;
- each bone's +Y points at its child, and `Left*` bones sit on +X of their `Right*` partners;
- rotating any one bone moves part of the mesh and nothing outside the region weighted to it and its children;
- seams stay closed with every bone bent at once.

Humanoid rigs additionally:
- the parent bones match `SkeletonProfileHumanoid` (extra bones such as a tail are listed, not failed);
- rotating `LeftUpperArm` moves the left arm but not the right arm or the feet.

## Limitations
- Quad-mode faces are roughly square (isotropic), so at 1k triangles a humanoid's limbs get about 5 sides even with adaptive spacing, and fine ridges are approximated more coarsely than voxel mode's long slivers would. Edge loops follow surface curvature, not the skeleton.
- Features thinner than about one quad, or about two voxels in voxel mode (fingers, ears, horn tips), can merge or disappear.
- Hard surface is a choice, not detected: telling planes, cylinders and doubly curved organic shapes apart depends too much on scale. Sharp edges smaller than about two voxels or half a quad aren't kept. Knife-thin parts (a roof that tapers to nothing) can tear the remesh in either mode.
- Parts that are just single-sided surfaces with nothing inside (capes, hair cards) end up as thin closed shells.
- Weights are automatic only; there's no weight painting. The humanoid template has no finger, eye or jaw bones.
- No texture baking: colour and surface detail from the source are not transferred. Paint the export in a PBR painter.
- Seams are placed for a model facing +Z; rotate other models on import (`--rotate-y`) for the best placement. Some seams stay where a chart reached its stretch limit, and they follow the stair-stepped borders of the starting charts there. Small cavities such as a mouth can keep a seam around them.
