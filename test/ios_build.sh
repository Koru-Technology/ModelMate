#!/bin/zsh
# Exports the iOS Xcode project (build/ios) and, with --sim, builds and launches it in an iPad simulator.
#   ./test/ios_build.sh            Xcode project only (open build/ios/ModelMate.xcodeproj to sign and run on a device)
#   ./test/ios_build.sh --sim      also build for the simulator, install, copy test models into the app's Files folder
#   ./test/ios_build.sh --sim --smoke   ...and run the UI smoke test inside the simulator (exit code = result)
#   TEAM_ID=XXXXXXXXXX ./test/ios_build.sh --device
#                                  build, sign (Xcode automatic signing, your team) and install on the connected iPad,
#                                  then copy the test models into the app's Files folder (Models/)
cd "${0:A:h}/.."
G=${GODOT:-/Applications/Godot.app/Contents/MacOS/Godot}
SIM=${SIM:-"iPad Pro 13-inch (M5)"}
BUNDLE=dev.warent.modelmate
set -e
mkdir -p build/ios && touch build/.gdignore   # keep Godot from importing build output
# Godot refuses to export without a team ID. Use $TEAM_ID if given, otherwise a placeholder: simulator builds are
# unsigned, and for a device you pick your Personal Team in Xcode (Signing & Capabilities), which replaces it.
cp export_presets.cfg build/ios/export_presets.cfg.bak
trap 'cp build/ios/export_presets.cfg.bak export_presets.cfg' EXIT
sed -i '' "s/^application\/app_store_team_id=.*/application\/app_store_team_id=\"${TEAM_ID:-0000000000}\"/" export_presets.cfg
$G --headless --path . --export-debug "iOS" build/ios/ModelMate.xcodeproj > build/ios/export.log 2>&1 || true
grep -E "ERROR|error CS|warning IL|warning: IL" build/ios/export.log | sort -u | head -40 || true
if grep -qE "ERROR: (Export .NET|Project export|Cannot export)|error CS" build/ios/export.log; then echo "export failed (see build/ios/export.log)"; exit 1; fi
[[ -d build/ios/ModelMate.xcodeproj ]] || { echo "export failed (see build/ios/export.log)"; exit 1; }
echo "Xcode project: build/ios/ModelMate.xcodeproj"
if [[ "$1" == "--device" ]]; then
  [[ -n "$TEAM_ID" ]] || { echo "set TEAM_ID (Xcode > Settings > Accounts > your team)"; exit 1; }
  DEST=$(xcodebuild -project build/ios/ModelMate.xcodeproj -scheme ModelMate -showdestinations 2>/dev/null \
    | grep -E "platform:iOS, arch" | head -1 | sed -E 's/.*id:([^,]+),.*/\1/')
  [[ -n "$DEST" ]] || { echo "no iOS device connected"; exit 1; }
  xcodebuild -project build/ios/ModelMate.xcodeproj -scheme ModelMate -configuration Debug -destination "id=$DEST" \
    -derivedDataPath build/ios/dddev DEVELOPMENT_TEAM=$TEAM_ID CODE_SIGN_STYLE=Automatic -allowProvisioningUpdates \
    build > build/ios/xcodebuild_dev.log 2>&1 || { grep -E "error:" build/ios/xcodebuild_dev.log | sort -u; exit 1; }
  xcrun devicectl list devices --json-output build/ios/devices.json > /dev/null 2>&1
  DEV=$(python3 -c "import json; d=json.load(open('build/ios/devices.json'))['result']['devices']; print(next(x['identifier'] for x in d if x['hardwareProperties'].get('deviceType')=='iPad' and x['connectionProperties'].get('tunnelState')!='unavailable'))")
  xcrun devicectl device install app --device $DEV build/ios/dddev/Build/Products/Debug-iphoneos/ModelMate.app > /dev/null
  for f in test/humanoid.glb test/lil_gug.glb; do
    [[ -f $f ]] && xcrun devicectl device copy to --device $DEV --domain-type appDataContainer \
      --domain-identifier $BUNDLE --source $f --destination Documents/Models/${f:t} > /dev/null
  done
  xcrun devicectl device copy to --device $DEV --domain-type appDataContainer --domain-identifier $BUNDLE \
    --source test/rigs/lil_gug.json --destination Documents/rigs/lil_gug.json > /dev/null
  echo "Installed on device $DEV"
  [[ "$2" == "--smoke" ]] || exit 0
  # UI smoke test on the device: arguments go in Documents/ui_smoke.txt; progress is read from the app's log
  RUN=$RANDOM$RANDOM
  # SMOKE_MODEL / SMOKE_RIG / SMOKE_TRIS pick the case (default: test humanoid, humanoid rig, 8000 triangles)
  printf -- "--ui-smoke\nuser://Models/${SMOKE_MODEL:-humanoid.glb}\nuser://Exports/smoke\n--rig\n${SMOKE_RIG:-humanoid}\n--tris\n${SMOKE_TRIS:-8000}\n--run-id\n$RUN\n" > build/ios/ui_smoke.txt
  dc() { xcrun devicectl device copy "$1" --device $DEV --domain-type appDataContainer --domain-identifier $BUNDLE \
           --source "$2" --destination "$3" > /dev/null 2>&1; }
  dc to build/ios/ui_smoke.txt Documents/ui_smoke.txt
  xcrun devicectl device process launch --terminate-existing --device $DEV $BUNDLE > /dev/null 2>&1
  rm -rf build/ios/godot.log build/ios/smoke
  for i in {1..90}; do
    sleep 5
    dc from Documents/logs/godot.log build/ios/godot.log || continue
    grep -q "Run id $RUN" build/ios/godot.log && grep -qE "UI smoke: (Done|Failed|.*FAILED)" build/ios/godot.log && break
  done
  mkdir -p build/ios/smoke
  for f in ui_heat.png ui_pose.png ui_export.glb; do dc from Documents/Exports/smoke/$f build/ios/smoke/$f; done
  grep -E "UI smoke|Test pose|\[(import|fit skeleton|retopology|weights|export)\]|FAIL|ERROR|Exception" build/ios/godot.log | grep -v "^ *at "
  grep -q "UI smoke: Done: export" build/ios/godot.log && [[ -f build/ios/smoke/ui_export.glb ]]
  exit $?
fi
[[ "$1" == "--sim" ]] || exit 0

# Godot's simulator library is x86_64 only, so the simulator app is x86_64 (Rosetta). iOS 26 simulator runtimes no
# longer install x86_64 apps on Apple silicon; use an iOS 18 (or older) simulator runtime: SIM="iPad Pro ... (iOS 18)".
xcodebuild -project build/ios/ModelMate.xcodeproj -scheme ModelMate -configuration Debug \
  -sdk iphonesimulator -destination "generic/platform=iOS Simulator" -derivedDataPath build/ios/dd \
  ARCHS=x86_64 ONLY_ACTIVE_ARCH=NO EXCLUDED_ARCHS=arm64 \
  CODE_SIGNING_ALLOWED=NO build > build/ios/xcodebuild.log 2>&1 || { tail -40 build/ios/xcodebuild.log; exit 1; }
APP=$(find build/ios/dd/Build/Products -name "*.app" -maxdepth 2 | head -1)
echo "Built $APP"
xcrun simctl boot "$SIM" 2>/dev/null || true
xcrun simctl install "$SIM" "$APP" || { echo "install failed: this simulator runtime can't run x86_64 apps (see above)"; exit 1; }
DATA=$(xcrun simctl get_app_container "$SIM" $BUNDLE data)
DOCS="$DATA/Documents"
mkdir -p "$DOCS/Models"
cp test/humanoid.glb "$DOCS/Models/"
[[ -f test/lil_gug.glb ]] && cp test/lil_gug.glb "$DOCS/Models/"
echo "App data: $DATA"
if [[ "$2" == "--smoke" ]]; then
  rm -rf "$DOCS/Exports/smoke"; mkdir -p "$DOCS/Exports/smoke"
  printf -- "--ui-smoke\nuser://Models/humanoid.glb\nuser://Exports/smoke\n" > "$DOCS/ui_smoke.txt"
  xcrun simctl launch --console-pty "$SIM" $BUNDLE > build/ios/sim_smoke.log 2>&1 &
  LPID=$!
  for i in {1..120}; do
    grep -q "UI smoke: \(Done\|Failed\)" build/ios/sim_smoke.log 2>/dev/null && break
    sleep 2
  done
  sleep 3; kill $LPID 2>/dev/null || true
  grep -E "UI smoke|PASS|FAIL|ERROR|memory|Files folder" build/ios/sim_smoke.log
  grep -q "UI smoke: Done: export" build/ios/sim_smoke.log && [[ -f "$DOCS/Exports/smoke/ui_export.glb" ]]
fi
