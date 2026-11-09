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

**import → retopologize (quad or voxel) → fit skeleton → auto skin weights → UV unwrap → bake albedo and normal maps → export**

Everything runs inside Godot, with no Blender or other external tools. With the humanoid rig, bone names and parent bones match Godot's `SkeletonProfileHumanoid`, which is what humanoid retargeting expects. To retarget, open the `.glb` in your game project's import dialog and set Skeleton3D → Retarget → Bone Map → `SkeletonProfileHumanoid`.

Speed on an M-series Mac, quad mode, 8,000 quads (16k triangles): about 4.5 s for a 46k-triangle model and about 8.5 s for a 3.8M-triangle one.

## Running

```sh
G=/Applications/Godot.app/Contents/MacOS/Godot
dotnet build                     # or let Godot build on launch
$G --path .                      # UI
```

**UI workflow** (right-hand panel, top to bottom):
1. Import a `.glb` or `.gltf`.
2. Click **Retopologize**. Quad mode is the default. In voxel mode, the voxel resolution sets the density and you can decimate to a triangle budget.
3. Click **Fit skeleton**, then left-drag the joints to adjust them. Left and right edits mirror each other by default. **Center joints in volume** re-centers the joints inside the mesh.
4. Click **Compute weights**. Tick *Show selected bone's weights* to see a heatmap, choosing the bone in the list or by clicking a joint.
5. Preview poses with the rotation sliders or **Test pose**. Use **Reset pose** before editing joints again.
6. Click **Export .glb**. This unwraps the UVs, bakes the maps, writes the file, then re-imports it and runs the checks listed below. The results appear in the log.

Camera: right-drag to orbit, Shift + right-drag to pan, mouse wheel or trackpad to zoom.

**Headless CLI** (the same pipeline, plus verification; exits 0 only if every check passes):

```sh
$G --headless --path . -- --in model.glb --out model_rigged.glb \
   [--retopo quad|voxel] [--quads 8000] [--dump-quads layout.obj] [--seed 1] \
   [--voxel 192] [--close 1] [--tris 8000] [--tex 1024] [--relax 4] [--falloff 3] \
   [--rig humanoid|single] [--rotate-y 180] [--no-normal] [--no-joint-aware] [--probe-weights]
$G --headless --path . -- --selftest-fields                      # quad remesher self-tests (sphere, off-axis capsule)
$G --headless --path . -- --make-test-asset res://test [--dense] # procedural test humanoids (T-pose, open, A-pose, ~3.8M-tri dense)
./test/regress.sh                                                # runs every test model through the CLI, both modes
$G --path . -- --render out.glb shot.png [--pose] [--wire]       # front/side snapshot (windowed); also accepts a quad .obj
$G --path . -- --ui-smoke res://test/humanoid.glb <dir>          # drives the UI end to end
```

## What the input should look like
- One character, upright: +Y up, **facing +Z**, character's left on +X. This is the glTF/Godot convention. If a model faces -Z, tick *Model faces -Z* before importing, or pass `--rotate-y 180`. Otherwise left and right get swapped, and the checks **can't detect this**, because they only compare the rig with itself.
- For the humanoid rig: T-pose or A-pose. The arm angle is detected from the bounding box.
- Use `--rig single` (or *Single bone* in the UI) for props.
- Vertex colours are baked as the glTF spec says (they multiply the base colour), even where Godot's own importer turns them off.

## How it works

| Stage | Method (`src/Core`) |
|---|---|
| Inside test | `VoxelGrid`: a narrow-band distance field. A point counts as inside if either test says so: a flood fill that first seals gaps (dilate, fill, erode), or a ray-crossing vote along X, Y and Z. This handles holes, overlapping parts and inverted normals. |
| Base mesh | `SurfaceNets`, then `MeshCleanup` (splits non-manifold edges and vertices), then `Relax` (smoothing along the surface plus guarded snapping to the source). This closed, evenly dense surface feeds both remesh modes and the normal-map bake. |
| Quad remesh (default) | `Quad/`: Instant Meshes port. A multiresolution hierarchy, a 4-RoSy orientation field and a 4-PoSy position field solved coarse to fine, then graph and face extraction, triangulation along each quad's shorter diagonal, hole filling and quad-preserving reprojection. If the result isn't closed and manifold or is under 70% quads, it falls back to voxel mode and the export checks fail, so a fallback is never silent. |
| Voxel remesh | The base mesh decimated by `QemDecimator`: quadric-error edge collapse with manifold and flip guards, up to 6× more detail kept near joints, and an edge-length cap. |
| Skeleton | `HumanoidTemplate`: fits to the bounding box, then centers joints inside the mesh by casting rays through it. Each bone's +Y points at its child. |
| Weights | `VoxelGeodesicWeights`: distance from each bone is measured *through the mesh interior* (Dijkstra over voxels), minus the bone's local thickness. Weight = 1/d^falloff. Weights are then smoothed, cut to the top 4 bones per vertex and normalized. They're computed on the welded mesh before unwrapping, so they're identical across UV seams. |
| UVs and bake | `UvUnwrap` (xatlas via `ArrayMesh.LightmapUnwrap`) and `TextureBaker`. Colour: a ray along the normal onto opaque, outward-facing source triangles, so transparent overlay shells and the backs of overlapping internal shells are ignored. Normals: from the clean base mesh, which avoids artifacts where source shells overlap. Detail finer than one base-mesh voxel isn't captured. Normal maps are tangent-space and OpenGL-style, using MikkTSpace tangents that are exported with the mesh. |
| Export | `RigBuilder` and `Exporter` (`GltfDocument`), then `Verify.RoundTrip`. |

## Checks run on export
The mesh is manifold. UVs are in [0,1]. Weights sum to 1 with at most 4 bones per vertex. Seam vertices carry identical attributes. In voxel mode the triangle budget is met.

In quad mode:
- no fallback was used;
- at least 90% quads and at least 85% valence-4 vertices;
- no boundary edges, and the same topology (Euler characteristic) as the base mesh;
- the face count is within ±20% of the target;
- surface deviation from the base mesh: mean under 0.15 and max under 0.75 of the quad spacing.

After re-import:
- the material has its albedo and normal textures;
- the rest pose reproduces the mesh.

Humanoid rig only:
- the bone names and parent bones match `SkeletonProfileHumanoid`;
- `LeftUpperArm` is on +X;
- each bone's +Y points at its child;
- rotating `LeftUpperArm` moves the left arm but not the right arm or the feet;
- seams stay closed under the pose.

## Limitations
- Quad mode spaces edges uniformly. Its edge loops follow surface curvature, not the skeleton, and there are no extra loops at joints yet.
- Features thinner than about one quad, or about two voxels in voxel mode (fingers, ears, horn tips), can merge or disappear.
- Parts that are just single-sided surfaces with nothing inside (capes, hair cards) end up as thin closed shells.
- Only the humanoid and single-bone rigs exist. The humanoid template has no finger, eye or jaw bones.
- Only albedo and normal maps are baked. Roughness and metallic maps are not.
- `LightmapUnwrap` needs an editor (tools) build, so run ModelMate through the Godot editor binary. Don't export it as a standalone app.
