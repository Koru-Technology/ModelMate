#!/bin/zsh
# Deformation report for the test models: quad vs voxel at several budgets, same weights for every variant.
# Writes <out>/<model>/report.md + report.csv + posed OBJs, and renders the named poses at mobile budgets.
cd "${0:A:h}/.."
G=${GODOT:-/Applications/Godot.app/Contents/MacOS/Godot}
OUT=${1:-test/report}
BUDGETS=${BUDGETS:-1000,2000,4000,8000,16000}
RENDER_BUDGETS=(${=RENDER_BUDGETS:-1000 2000})
fail=0
for spec in "lil_gug:res://test/rigs/lil_gug.json" "humanoid_apose:humanoid" "humanoid_lumpy:humanoid"; do
  model=${spec%%:*}; rig=${spec#*:}
  [[ -f test/$model.glb ]] || { echo "$model: skipped (test/$model.glb missing)"; continue; }
  dir=$OUT/$model; mkdir -p $dir
  echo "== $model"
  $G --headless --path . -- --in res://test/$model.glb --rig $rig --deform-report $dir --budgets $BUDGETS > $dir/log.txt 2>&1 || fail=1
  grep -c "FALLBACK" $dir/log.txt | sed 's/^/fallbacks: /'
  for b in $RENDER_BUDGETS; do
    for f in $dir/quad${b}_*.obj $dir/voxel${b}_*.obj; do
      [[ -f $f ]] || continue
      $G --path . --resolution 500x500 -- --render $f ${f%.obj}.png --views front,side,three > /dev/null 2>&1
    done
  done
  echo "report: $dir/report.md"
done
exit $fail
