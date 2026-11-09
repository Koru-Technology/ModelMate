#!/bin/zsh
# Budget sweep: each model through the quad pipeline at several triangle budgets.
# Prints one row per run; exit 1 if any run fails a check. Models that aren't present are skipped.
cd "${0:A:h}/.."
G=${GODOT:-/Applications/Godot.app/Contents/MacOS/Godot}
BUDGETS=(${=BUDGETS:-1000 2000 4000 8000 16000})
fail=0
printf "%-20s %6s  %4s  %-9s %6s %9s %10s  %s\n" model tris exit fallback quads% irregular out-tris failed-checks
for spec in "humanoid_apose:humanoid" "humanoid_lumpy:humanoid" "lil_gug:single"; do
  model=${spec%%:*}; rig=${spec##*:}
  [[ -f test/$model.glb ]] || { echo "$model: skipped (test/$model.glb missing)"; continue; }
  for t in $BUDGETS; do
    out=$($G --headless --path . -- --in res://test/$model.glb --out res://test/out_sweep.glb --tris $t --rig $rig 2>&1); rc=$?
    fb=$(echo $out | grep -q "falling back" && echo yes || echo no)
    q=$(echo $out | grep -o "Quad mesh: .*" | grep -o "quad%=[0-9,.]*%" | head -1 | sed 's/quad%=//')
    irr=$(echo $out | grep -o "Quad mesh: .*" | grep -o "irregular=[0-9]*" | head -1 | sed 's/irregular=//')
    ot=$(echo $out | grep -o "Exported .*" | grep -o "([0-9]* tris" | grep -o "[0-9]*")
    fc=$(echo $out | grep "\[FAIL\]" | sed 's/.*\[FAIL\] //' | cut -c1-60 | paste -sd ';' -)
    printf "%-20s %6s  %4s  %-9s %6s %9s %10s  %s\n" $model $t $rc $fb "${q:--}" "${irr:--}" "${ot:--}" "$fc"
    (( rc != 0 )) && fail=1
  done
done
rm -f test/out_sweep.glb
exit $fail
