#!/bin/zsh
# Runs the CLI pipeline (with verification) over the procedural test models. Exit 1 if any case fails.
cd "${0:A:h}/.."
G=${GODOT:-/Applications/Godot.app/Contents/MacOS/Godot}
[[ -f test/humanoid.glb ]] || $G --headless --path . -- --make-test-asset res://test --dense >/dev/null 2>&1
fail=0
run() {
  local out rc
  out=$($G --headless --path . -- "$@" 2>&1); rc=$?
  echo "$* -> exit=$rc $(echo $out | grep -o 'Exported.*' | sed 's/.*glb //')"
  echo $out | grep -E "FAIL|ERROR|WARNING"
  (( rc != 0 )) && fail=1
}
run --selftest-fields
for f in humanoid humanoid_open humanoid_apose humanoid_lumpy humanoid_dense; do
  run --in res://test/$f.glb --out res://test/out_reg.glb                      # quad mode (default, 8000 triangles)
done
run --in res://test/humanoid.glb --out res://test/out_reg.glb --quads 3000
run --in res://test/humanoid_lumpy.glb --out res://test/out_reg.glb --tris 1000   # mobile budget
if [[ -f test/lil_gug.glb ]]; then
  run --in res://test/lil_gug.glb --out res://test/out_reg.glb --tris 2000 --rig res://test/rigs/lil_gug.json
  run --in res://test/lil_gug.glb --out res://test/out_reg.glb --rig blob_biped        # built-in preset fits from scratch
fi
run --in res://test/humanoid_apose.glb --out res://test/out_reg.glb --tris 2000 --unwrap charts   # unwrapper used by exported apps (iPad)
run --in res://test/humanoid_open.glb --out res://test/out_reg.glb --retopo voxel --voxel 128 --tris 3000 --unwrap charts
[[ -f test/lil_gug.glb ]] && run --in res://test/lil_gug.glb --out res://test/out_reg.glb --tris 2000 --rig res://test/rigs/lil_gug.json --unwrap charts
run --in res://test/humanoid.glb --out res://test/out_reg.glb --rig single
run --in res://test/humanoid.glb --out res://test/out_reg.glb --retopo voxel --tris 8000
run --in res://test/humanoid_open.glb --out res://test/out_reg.glb --retopo voxel --voxel 128 --tris 3000
rm -f test/out_reg.glb test/out_q.glb
exit $fail
