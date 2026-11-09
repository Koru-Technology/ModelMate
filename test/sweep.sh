#!/bin/zsh
# Budget sweep: each model at several triangle budgets, quad mode, plus voxel mode at mobile budgets for comparison.
# One row per run; exit 1 if any run fails a check. Models that aren't present are skipped.
# Note: when piping this script, read its status from $pipestatus[1], not $?.
# EXTRA="--curv 1 ..." passes extra CLI flags to every run.
cd "${0:A:h}/.."
G=${GODOT:-/Applications/Godot.app/Contents/MacOS/Godot}
BUDGETS=(${=BUDGETS:-1000 2000 4000 8000 16000})
VOXEL_BUDGETS=(${=VOXEL_BUDGETS:-1000 2000})
fail=0
row() {  # model rig mode tris
  local out rc
  out=$($G --headless --path . -- --in res://test/$1.glb --out res://test/out_sweep.glb --rig $2 --retopo $3 --tris $4 ${=EXTRA} 2>&1); rc=$?
  local fb=$(echo $out | grep -q "falling back" && echo yes || echo no)
  local q=$(echo $out | grep -o "quad share [0-9,.]*%" | head -1 | sed 's/quad share //')
  local sz=$(echo $out | grep -o "spacing: [a-z]*" | head -1 | sed 's/spacing: //')
  local v4=$(echo $out | grep -o "valence-4 vertices [0-9,.]*%" | head -1 | sed 's/valence-4 vertices //')
  local sides=$(echo $out | grep -o "limb sides: min [0-9]*" | grep -o "[0-9]*$")
  local ot=$(echo $out | grep -o "Exported .*" | grep -o "([0-9]* tris" | grep -o "[0-9]*")
  local warn=$(echo $out | grep -c "\[WARN\]")
  local fc=$(echo $out | grep "\[FAIL\]" | sed 's/.*\[FAIL\] //' | cut -c1-70 | paste -sd ';' -)
  printf "%-16s %-6s %6s %4s %-4s %-8s %6s %6s %5s %7s %4s  %s\n" $1 $3 $4 $rc $fb "${sz:--}" "${q:--}" "${v4:--}" "${sides:--}" "${ot:--}" $warn "$fc"
  (( rc != 0 )) && fail=1
}
printf "%-16s %-6s %6s %4s %-4s %-8s %6s %6s %5s %7s %4s  %s\n" model mode tris exit fb spacing quads% val4% sides outTris warn failed-checks
for spec in "humanoid_apose:humanoid" "humanoid_lumpy:humanoid" "lil_gug:res://test/rigs/lil_gug.json"; do
  model=${spec%%:*}; rig=${spec#*:}
  [[ -f test/$model.glb ]] || { echo "$model: skipped (test/$model.glb missing)"; continue; }
  for t in $BUDGETS; do row $model $rig quad $t; done
  for t in $VOXEL_BUDGETS; do row $model $rig voxel $t; done
done
rm -f test/out_sweep.glb
exit $fail
