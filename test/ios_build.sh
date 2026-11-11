#!/bin/zsh
# Exports the iOS Xcode project (build/ios) and, with --sim, builds and launches it in an iPad simulator.
#   ./test/ios_build.sh            Xcode project only (open build/ios/ModelMate.xcodeproj to sign and run on a device)
#   ./test/ios_build.sh --sim      also build for the simulator, install, copy test models into the app's Files folder
#   ./test/ios_build.sh --sim --smoke   ...and run the UI smoke test inside the simulator (exit code = result)
cd "${0:A:h}/.."
G=${GODOT:-/Applications/Godot.app/Contents/MacOS/Godot}
SIM=${SIM:-"iPad Pro 13-inch (M5)"}
BUNDLE=dev.warent.modelmate
set -e
mkdir -p build/ios
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
