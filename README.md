# ModelMate

## Credits: projects, libraries and papers this builds on

**Code ported into this project**
- **[Instant Meshes](https://github.com/wjakob/instant-meshes)** (BSD 3-Clause) by Wenzel Jakob, Marco Tarini, Daniele Panozzo and Olga Sorkine-Hornung: *Instant Field-Aligned Meshes*, ACM Transactions on Graphics (SIGGRAPH Asia) 2015. The quad remesher in `src/Core/Quad/` (`FieldMath`, `Hierarchy`, `FieldSolver`, `Extraction`) is a C# port of its field optimization and mesh extraction. The full license is in [`THIRD_PARTY.md`](THIRD_PARTY.md).

**Libraries used (through Godot, not vendored here)**
- **[Godot Engine](https://godotengine.org) 4.7** (MIT): the runtime, UI, rendering, glTF import and export (`GLTFDocument`), `Skeleton3D`/`Skin`, and `SkeletonProfileHumanoid` (the bone names and hierarchy of the humanoid rig).
- **[.NET 8](https://dotnet.microsoft.com) and the Godot .NET SDK** (MIT): the C# toolchain.
- **[xatlas](https://github.com/jpcy/xatlas)** (MIT): UV unwrapping, called through Godot's `ArrayMesh.LightmapUnwrap`.
- **[MikkTSpace](https://github.com/mmikk/MikkTSpace)** (zlib-style license) by Morten Mikkelsen: tangent generation for normal maps, called through Godot's `SurfaceTool.GenerateTangents`.

**Algorithms implemented from publications (no code copied)**
- **Surface Nets**: S. Gibson, *Constrained Elastic Surface Nets*, MICCAI 1998. Used in the voxel remesher (`SurfaceNets.cs`).
- **Quadric error metrics**: M. Garland and P. Heckbert, *Surface Simplification Using Quadric Error Metrics*, SIGGRAPH 1997. Used for decimation (`QemDecimator.cs`).
- **Geodesic voxel skinning**: the idea of measuring bone distance through the voxelized interior comes from O. Dionne and M. de Lasa, *Geodesic Voxel Binding for Production Character Meshes*, SCA 2013. `VoxelGeodesicWeights.cs` is a simplified variant (Dijkstra over interior voxels with an inverse-distance falloff), not their full method.
- **Closest point on a triangle**: C. Ericson, *Real-Time Collision Detection*, 2004 (`Geo.ClosestPtTri`).
- **Ray–triangle intersection**: T. Möller and B. Trumbore, *Fast, Minimum Storage Ray/Triangle Intersection*, 1997 (`Geo.RayTri`).

**Tools used only during development (not part of the app)**
- **[Pillow](https://python-pillow.org) and [NumPy](https://numpy.org)** (Python): compared rendered screenshots when checking the normal-map bake.
- **curl**: downloaded the Instant Meshes reference source for porting.

---

A small Godot 4.7 (C#) tool. It turns an unrigged static glTF (a dense sculpt or AI-generated mesh) into a game-ready rigged `.glb`:

**import → fit skeleton (humanoid, creature or your own) → retopologize (quad or voxel) → auto skin weights → UV unwrap → bake albedo and normal maps → export**

Everything runs inside Godot, with no Blender or other external tools. With the humanoid rig, bone names and parent bones match Godot's `SkeletonProfileHumanoid`, which is what humanoid retargeting expects. To retarget, open the `.glb` in your game project's import dialog and set Skeleton3D → Retarget → Bone Map → `SkeletonProfileHumanoid`.

Speed on an M-series Mac, quad mode, at the default budget of 8,000 triangles: about 3–5 s for a 46k-triangle model and about 8 s for a 3.8M-triangle one.

## Running

```sh
G=/Applications/Godot.app/Contents/MacOS/Godot
dotnet build                     # or let Godot build on launch
$G --path .                      # UI
```

**UI workflow** (right-hand panel, top to bottom):
1. Import a `.glb` or `.gltf`.
2. **Skeleton.** Pick a rig preset and click **Fit skeleton**, then left-drag joints to adjust them (left/right edits mirror by default). Edit the skeleton with **Add child**, **Delete**, **Rename**, **Parent**, **Deforms mesh** and **Symmetrize** (copies every `Left*` bone onto its `Right*` partner). **Save** stores it as a preset you can reuse on other models; **Load preset file…** fits a preset `.json` from anywhere.
3. Pick a mode and a **target triangle count**, then click **Retopologize**. Quad mode is the default. In voxel mode the voxel resolution sets the detail and the result is decimated to the target.
4. Click **Compute weights**. Tick *Show selected bone's weights* to see a heatmap, choosing the bone in the list or by clicking a joint.
5. Preview poses with the rotation sliders or **Test pose**. Use **Reset pose** before editing joints again.
6. Click **Export .glb**. This unwraps the UVs, bakes the maps, writes the file, then re-imports it and runs the checks listed below. The results appear in the log.

Camera: right-drag to orbit, Shift + right-drag to pan, mouse wheel or trackpad to zoom.

**Headless CLI** (the same pipeline, plus verification; exits 0 only if every check passes):

```sh
$G --headless --path . -- --in model.glb --out model_rigged.glb \
   [--retopo quad|voxel] [--tris 8000] [--sizing auto|uniform|adaptive] [--detail 256] [--flow 0] \
   [--dump-quads layout.obj] [--seed 1] \
   [--voxel 256] [--close 1] [--tex 1024] [--relax 4] [--falloff 3] \
   [--rig humanoid|single|blob_biped|preset.json] [--save-rig out.json] \
   [--rotate-y 180] [--no-normal] [--no-joint-aware] [--probe-weights]
$G --headless --path . -- --in model.glb --rig humanoid --deform-report <dir> [--budgets 1000,2000] [--modes quad,quadflow,voxel]
$G --headless --path . -- --selftest-fields                      # quad remesher + rig editing self-tests
$G --headless --path . -- --make-test-asset res://test [--dense] # procedural test humanoids (T-pose, open, A-pose, ~3.8M-tri dense)
./test/regress.sh                                                # runs every test model through the CLI, both modes
./test/sweep.sh                                                  # quality table across budgets (1k–16k triangles)
$G --path . -- --render out.glb shot.png [--views front,side,back,top,bottom,three] [--wire] \
   [--pose | --pose-preset rigs/blob_biped.json waddle_left]     # snapshot (windowed); also accepts a quad .obj
$G --path . -- --ui-smoke res://test/humanoid.glb <dir>          # drives the UI end to end
```

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
- Vertex colours are baked as the glTF spec says (they multiply the base colour), even where Godot's own importer turns them off.

## How it works

| Stage | Method (`src/Core`) |
|---|---|
| Inside test | `VoxelGrid`: a narrow-band distance field. A point counts as inside if either test says so: a flood fill that first seals gaps (dilate, fill, erode), or a ray-crossing vote along X, Y and Z. This handles holes, overlapping parts and inverted normals. |
| Base mesh | `SurfaceNets`, then `MeshCleanup` (splits non-manifold edges and vertices), then `Relax` (smoothing along the surface plus guarded snapping to the source). This closed, evenly dense surface feeds both remesh modes and the normal-map bake. |
| Quad remesh (default) | `Quad/`: Instant Meshes port. A multiresolution hierarchy, a 4-RoSy orientation field and a 4-PoSy position field solved coarse to fine, then graph and face extraction, triangulation along each quad's shorter diagonal, hole filling and quad-preserving reprojection. If the result isn't closed and manifold or is under 70% quads, it falls back to voxel mode and the export checks fail, so a fallback is never silent. |
| Voxel remesh | The base mesh decimated by `QemDecimator`: quadric-error edge collapse with manifold and flip guards, up to 6× more detail kept near joints, and an edge-length cap. |
| Skeleton | `RigPreset` fits a preset to the bounding box (`HumanoidTemplate` for the humanoid), then centers joints inside the mesh by casting rays through it. `RigEdit` does the structural edits and keeps the skeleton valid (one root, parents before children, unique names, no cycles). Each bone's +Y points at its child. |
| Weights | `VoxelGeodesicWeights`: distance from each bone is measured *through the mesh interior* (Dijkstra over voxels), minus the bone's local thickness. Weight = 1/d^falloff. They're solved once on the dense base surface, smoothed over a world-space radius (2% of model height), mirror-averaged when the model and rig are both symmetric (`Symmetry`), cut to the top 4 bones and normalized, then transferred to the output mesh by closest point. So weights don't change with the triangle budget or the remesh, and they're identical across UV seams. |
| UVs and bake | `UvUnwrap` (xatlas via `ArrayMesh.LightmapUnwrap`) and `TextureBaker`. Colour: a ray along the normal onto opaque, outward-facing source triangles, so transparent overlay shells and the backs of overlapping internal shells are ignored. Normals: from the clean base mesh, which avoids artifacts where source shells overlap. Detail finer than one base-mesh voxel isn't captured. Normal maps are tangent-space and OpenGL-style, using MikkTSpace tangents that are exported with the mesh. |
| Export | `RigBuilder` and `Exporter` (`GltfDocument`), then `Verify.RoundTrip`. |

## Deformation report: quad vs voxel

`./test/deform_report.sh [outdir]` (or `--deform-report <dir>` on the CLI) poses every variant (mode × budget) with the *same* weights, transferred from a dense reference, so the numbers measure tessellation and edge layout only. It writes `report.md`, `report.csv` and posed OBJs per model, and renders the named poses at mobile budgets.

What it shows on the test models (Lil gug, the A-pose humanoid and the lumpy humanoid, 1k–16k triangles):

Average p99 error over the bone poses, % of model height, defaults for both modes (quad: auto spacing; both: surface fit):

| model | | 1k | 2k | 4k | 8k |
|---|---|---|---|---|---|
| A-pose humanoid | total posed (quad / voxel) | **0.63** / 0.68 | 0.35 / **0.32** | 0.24 / 0.24 | **0.16** / 0.21 |
| | bending | **0.25** / 0.36 | **0.16** / 0.24 | **0.11** / 0.23 | **0.07** / 0.21 |
| lumpy humanoid | total posed | **0.62** / 0.65 | 0.39 / **0.29** | 0.25 / **0.22** | 0.17 / **0.16** |
| | bending | **0.26** / 0.32 | **0.17** / 0.24 | **0.11** / 0.20 | **0.09** / 0.15 |
| Lil gug | total posed | 1.36 / **1.02** | 0.74 / 0.74 | **0.49** / 0.54 | 0.38 / 0.38 |
| | bending | **0.80** / 0.92 | **0.62** / 0.69 | **0.44** / 0.52 | **0.36** / 0.36 |

*Bending* is how much worse the mesh gets when posed; *total posed* is what a viewer sees in the pose (shape error plus bending).

- **Quad mode bends more cleanly at every budget.** Voxel mode's bending error stays high even at 8k because decimation leaves long slivers that span joints.
- **Overall they're close**: quad wins or ties in 7 of 12 cells. Voxel mode still matches shape better in a few (Lil gug at 1k, fine ridges at 2k–4k), because its long slivers follow ridges that roughly square quads can't.
- Quad stays the default because it's the mode that animates cleanly; voxel mode is there when a static silhouette matters more.

How quad mode got here, measured with the same report:
1. **Uniform spacing** was worse at matching shape at mobile budgets (e.g. 1.19 vs voxel 0.75 total on Lil gug at 2k), because triangles were spread by area, not by need.
2. **Adaptive spacing** (`--sizing auto`, below) spends more quads where the surface curves.
3. **Surface fitting** (`SurfaceFit`, on by default in both modes, `--no-fit` to disable) moves vertices slightly off the surface along their normals so the faces fit the dense surface in a least-squares sense. A coarse mesh with every vertex *on* a curved surface has every face cutting inside it; balancing that roughly halves shape error. It took Lil gug at 2k from 0.86 to 0.74 in quad mode and the A-pose humanoid at 1k from 1.01 to 0.68 in voxel mode.

(The report builds both modes from the same 256-cell base mesh, which is also the shipping default for both.)

**Adaptive spacing** (quad mode, default `--sizing auto`). A sizing field gives each point of the surface a target edge length from its curvature (spacing ∝ κ^-½, the rule for constant chord error), limited to a 6× range and to gradual changes (≤ 0.2 per unit distance), and rescaled so the total meets the triangle budget (one recalibration pass if extraction misses it by more than 4%). `auto` builds both the uniform and the adaptive remesh from the same base mesh and keeps the one closer to the surface (99th-percentile deviation), so shape fidelity never gets worse; it costs one or two extra remesh passes. Adaptive spacing puts its triangles where density changes, so the quad share drops (83–94% at 1k–4k, vs 93–97% uniform). A thickness term (`--adapt 0.5`) is available but measured as a wash next to curvature alone.

**Skeleton-guided edge flow** (`--flow 0.7`, quad mode) pulls the field toward each region's bone axis, with extra weight at joints. Measured with the same report at 1k–4k triangles it didn't earn a place as the default: it cut collapsed triangles on the A-pose humanoid at 1k (1.2 → 0.7 per 1,000) but made total posed error slightly worse on Lil gug at every budget (by up to 8%) and on the lumpy humanoid at 1k and 4k. On these models the unguided field already runs along the limbs, so guidance mostly adds singularities. It stays available as an experimental CLI option.

## Checks run on export
The mesh is manifold. UVs are in [0,1]. Weights sum to 1 with at most 4 bones per vertex. Seam vertices carry identical attributes. In voxel mode the triangle budget is met.

In quad mode:
- no fallback was used;
- at least 80% quads (catches broken extraction; uniform spacing typically gives ≥93%, adaptive 83–94%);
- no boundary edges, and the same topology (Euler characteristic) as the base mesh;
- the face count is within ±20% of the target;
- surface deviation from the base mesh: mean under 0.15 and 99.5th percentile under 0.5 of the quad spacing. A larger worst-case deviation (typically a thin tip cut short) is reported as a `[WARN]` with its location.

Also reported as `[METRIC]` lines, without failing the run: the share of valence-4 vertices, the worst surface deviation, and (humanoid rig) the number of sides around each limb.

After re-import, for any rig:
- the material has its albedo and normal textures;
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
- Parts that are just single-sided surfaces with nothing inside (capes, hair cards) end up as thin closed shells.
- Weights are automatic only; there's no weight painting. The humanoid template has no finger, eye or jaw bones.
- Only albedo and normal maps are baked. Roughness and metallic maps are not.
- `LightmapUnwrap` needs an editor (tools) build, so run ModelMate through the Godot editor binary. Don't export it as a standalone app.
