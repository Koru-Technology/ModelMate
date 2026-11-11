using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Godot;
using ModelMate.Core;

namespace ModelMate.App;

/// <summary>
/// ModelMate UI. Until a model is loaded: one big Import button. Then an icon toolbar in the top-left corner (import,
/// retopology, export, view and settings menus, console), frame + orientation cube at the top right of the view, the
/// Retopology window (opens after each import), and a side panel of collapsible, optional rigging sections
/// (Skeleton & weights, Preview pose). Export needs only a retopologized mesh.
/// Heavy steps run on worker threads; Godot objects are only touched on the main thread.
/// With command-line user args, runs the headless CLI (see Cli) or the snapshot renderer instead.
/// </summary>
public partial class Main : Node3D
{
    readonly Pipeline _p = new();
    readonly ConcurrentQueue<string> _logQ = new();

    OrbitCamera _orbit;
    BoneGizmo _gizmo;
    Node3D _sourceNode;
    MeshInstance3D _retopoMI;
    Node3D _previewRoot;
    Skeleton3D _previewSkel;
    MeshInstance3D _previewMI;

    // UI
    RichTextLabel _log;
    Label _status, _voxelInfo, _triInfo;
    SpinBox _voxel, _close, _tris, _relax, _falloff, _tex;
    CheckBox _mirror, _heat;
    MenuToggle _faceNegZ, _showSource, _showRetopo, _wire, _showSkel;
    int _texSizeValue = 1024;
    OptionButton _retopoMode, _spacing, _rigPreset, _parentList, _boneList;
    HSlider _rx, _ry, _rz;
    readonly List<Button> _actionButtons = new();
    Button _btnRetopo, _btnFit, _btnCenter, _btnWeights, _btnExport, _btnTestPose, _btnResetPose;
    Button _btnAddBone, _btnDelBone, _btnSym, _btnRename, _btnSavePreset, _btnLoadPreset;
    LineEdit _renameEdit, _presetName;
    CheckBox _deformCheck;
    bool _syncingBoneUi;
    string _sourcePath;
    bool _busy, _weightsStale = true;
    readonly Dictionary<int, Vector3> _poseEuler = new();

    public override async void _Ready()
    {
        var args = OS.GetCmdlineUserArgs();
        int ri = Array.IndexOf(args, "--render");
        if (ri >= 0 && ri + 2 < args.Length)
        {
            int vi = Array.IndexOf(args, "--views");
            int rc = await Render.Run(this, args[ri + 1], args[ri + 2],
                Array.IndexOf(args, "--pose") >= 0, Array.IndexOf(args, "--wire") >= 0,
                vi >= 0 && vi + 1 < args.Length ? args[vi + 1] : "front,side",
                Array.IndexOf(args, "--pose-preset") is int pp and >= 0 && pp + 2 < args.Length ? args[pp + 1] : null,
                Array.IndexOf(args, "--pose-preset") is int pq and >= 0 && pq + 2 < args.Length ? args[pq + 2] : null);
            GetTree().Quit(rc);
            return;
        }
        _mobile = OS.HasFeature("mobile");
        if (_mobile && args.Length == 0) args = SmokeArgsFromFile();
        int si = Array.IndexOf(args, "--ui-smoke");
        bool touchTest = args.Contains("--touch-test");
        _touchUi = _mobile || args.Contains("--touch-ui") || touchTest;
        if (args.Length > 0 && si < 0 && !_touchUi)
        {
            GetTree().Quit(Cli.Run(args));
            return;
        }
        _p.Log = s => _logQ.Enqueue(s);
        if (_touchUi) SetupTouch();
        BuildWorld();
        BuildUi();
        UpdateButtons();
        if (Array.IndexOf(args, "--run-id") is int ri2 and >= 0 && ri2 + 1 < args.Length) GD.Print($"Run id {args[ri2 + 1]}");
        if (si >= 0 && si + 2 < args.Length)
        {
            string rig = Array.IndexOf(args, "--rig") is int rgi and >= 0 && rgi + 1 < args.Length ? args[rgi + 1] : "humanoid";
            int tris = Array.IndexOf(args, "--tris") is int tri and >= 0 && tri + 1 < args.Length ? int.Parse(args[tri + 1]) : 8000;
            await UiSmoke(args[si + 1], args[si + 2], rig, tris);
            return;
        }
        if (touchTest) { await TouchTest(); return; }
        Log(_touchUi
            ? "Put .glb files in Files › On My iPad › ModelMate › Models, then tap Import. One finger: orbit · two fingers: pinch to zoom, drag to pan · drag a joint to move it."
            : "Import a .glb/.gltf to begin. Left/right-drag: orbit · Shift+drag: pan · Wheel: zoom · Left-drag joints to edit.");
    }

    // ------------------------------------------------------------------ touch / mobile

    bool _mobile, _touchUi;
    long _peakMem;
    double _memSampleT;

    /// <summary>
    /// Mobile has no command line, so automated runs (simulator tests) put the arguments in user://ui_smoke.txt,
    /// one per line. The file is consumed so the next launch starts normally.
    /// </summary>
    static string[] SmokeArgsFromFile()
    {
        const string f = "user://ui_smoke.txt";
        if (!Godot.FileAccess.FileExists(f)) return Array.Empty<string>();
        var lines = Godot.FileAccess.GetFileAsString(f).Split('\n', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        DirAccess.RemoveAbsolute(ProjectSettings.GlobalizePath(f));
        return lines;
    }

    /// <summary>UI scale for high-DPI touch screens, the app's Files folders, and a larger joint pick radius.</summary>
    void SetupTouch()
    {
        if (_mobile)
        {
            float scale = MathF.Max(1, DisplayServer.ScreenGetScale());
            GetWindow().ContentScaleFactor = scale;
        }
        foreach (var d in new[] { "user://Models", "user://Exports", "user://rigs" })
            DirAccess.MakeDirRecursiveAbsolute(ProjectSettings.GlobalizePath(d));
        Log($"Files folder: {OS.GetUserDataDir()}");
        if (_mobile)
        {
            IosBridge.Register();   // normally already done when the assembly loaded
            if (IosBridge.RegisterError != null) Log("Receiving shared files is unavailable: " + IosBridge.RegisterError);
            IosBridge.SweepInbox();
        }
    }

    /// <summary>Touch-sized controls (44 pt) everywhere under the panel.</summary>
    static void EnlargeForTouch(Node n)
    {
        if (n is Button or SpinBox or LineEdit or HSlider)
        {
            var c = (Control)n;
            c.CustomMinimumSize = new Vector2(c.CustomMinimumSize.X, MathF.Max(c.CustomMinimumSize.Y, 44));
        }
        foreach (var ch in n.GetChildren()) EnlargeForTouch(ch);
    }

    void SampleMemory()
    {
        try { _peakMem = Math.Max(_peakMem, MemoryProbe.FootprintBytes()); } catch { /* not supported */ }
    }

    string MemoryText()
    {
        try
        {
            long avail = MemoryProbe.AvailableBytes();
            return $"memory {MemoryProbe.FootprintBytes() / 1048576} MB, peak {_peakMem / 1048576} MB" +
                   (avail >= 0 ? $", {avail / 1048576} MB more allowed by iOS" : "");
        }
        catch { return "memory n/a"; }
    }

    int _testPoseIdx;

    /// <summary>
    /// Shows a test pose: the preset's named poses in turn (e.g. waddle_left, waddle_right for blob rigs), or the
    /// standard humanoid bend. Returns a log line.
    /// </summary>
    string ApplyNextTestPose()
    {
        if (_previewSkel == null) return "Compute weights first.";
        var poses = _p.RigPresetData?.Poses;
        if (poses != null && poses.Count > 0)
        {
            var names = poses.Keys.OrderBy(k => k).ToList();
            var name = names[_testPoseIdx++ % names.Count];
            ResetPose();
            Render.ApplyPresetPose(_previewSkel, _p.RigPresetData, name);
            return $"Test pose: {name}";
        }
        Render.ApplyTestPose(_previewSkel);
        return "Test pose: humanoid bend";
    }

    /// <summary>Drives the UI steps programmatically (same code paths as the buttons), then screenshots.</summary>
    async Task UiSmoke(string input, string outDir, string rig = "humanoid", int tris = 8000)
    {
        outDir = ProjectSettings.GlobalizePath(outDir);
        System.IO.Directory.CreateDirectory(outDir);
        await Shot(System.IO.Path.Combine(outDir, "ui_empty.png"));
        await Import(ProjectSettings.GlobalizePath(input));
        await Shot(System.IO.Path.Combine(outDir, "ui_source.png"));
        _tris.Value = tris;
        int ri = RigPreset.List().IndexOf(rig);
        if (ri >= 0) { _rigPreset.Select(ri); await OnFitPressed(); }
        else await FitWith(rig);
        var retopo = OnRetopoPressed();
        await Shot(System.IO.Path.Combine(outDir, "ui_busy.png"));   // the progress overlay while it runs
        await retopo;
        await Shot(System.IO.Path.Combine(outDir, "ui_retopo.png"));
        await OnWeightsPressed();
        // humanoid: upper arm / hand; other rigs: the first Left* bone
        string heatBone = _p.Rig.Find("LeftUpperArm") >= 0 ? "LeftUpperArm" : _p.Rig.Bones.First(b => b.Name.StartsWith("Left")).Name;
        string editBone = _p.Rig.Find("LeftHand") >= 0 ? "LeftHand" : heatBone;
        _boneList.Select(_p.Rig.Find(heatBone));
        _gizmo.Select(_p.Rig.Find(heatBone));
        _heat.ButtonPressed = true;
        OnBoneSelected(_gizmo.Selected);
        await Shot(System.IO.Path.Combine(outDir, "ui_heat.png"));
        _heat.ButtonPressed = false;
        Log(ApplyNextTestPose());
        await Shot(System.IO.Path.Combine(outDir, "ui_pose.png"));
        ResetPose();
        // skeleton editor through the same handlers as the buttons
        int hand = _p.Rig.Find(editBone);
        _boneList.Select(hand); _gizmo.Select(hand); OnBoneSelected(hand);
        OnAddBone();                                                   // child of the Left* bone
        _renameEdit.Text = "LeftClaw"; OnRenameBone();
        OnSymmetrize();                                                // creates RightClaw
        bool editsOk = _p.Rig.Find("LeftClaw") >= 0 && _p.Rig.Find("RightClaw") >= 0 &&
                       _p.Rig.Bones[_p.Rig.Find("RightClaw")].Parent == _p.Rig.Find("Right" + editBone[4..]);
        _presetName.Text = "ui_smoke_rig"; OnSavePreset();
        await FitWith("ui_smoke_rig");                                 // reload the saved preset
        editsOk &= _p.Rig.Find("RightClaw") >= 0;
        GD.Print($"UI smoke: skeleton edits {(editsOk ? "ok" : "FAILED")}");
        var saved = RigPreset.Resolve("ui_smoke_rig");
        if (saved != null) System.IO.File.Delete(saved);
        await ExportTo(System.IO.Path.Combine(outDir, "ui_export.glb"));
        while (_logQ.Count > 0) await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
        bool ok = _status.Text.StartsWith("Done") && _lastExportOk == true && editsOk;
        GD.Print("UI smoke: " + _status.Text);
        GetTree().Quit(ok ? 0 : 1);
    }

    /// <summary>
    /// Feeds synthetic touch events through Input.ParseInputEvent (the same path a touch screen uses, including
    /// Godot's touch-to-mouse emulation) and checks orbit, joint drag and pinch. Exit code 0 when all pass.
    /// </summary>
    async Task TouchTest()
    {
        var ok = true;
        void Check(bool c, string msg) { GD.Print((c ? "  [PASS] " : "  [FAIL] ") + msg); ok &= c; }
        async Task Frames(int n = 3) { for (int i = 0; i < n; i++) await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame); }
        // positions below are in UI points; touch events carry window pixels (UI scale 2 on an iPad)
        float px = GetWindow().ContentScaleFactor;
        async Task Touch(int idx, Vector2 pos, bool down) { Input.ParseInputEvent(new InputEventScreenTouch { Index = idx, Position = pos * px, Pressed = down }); await Frames(); }
        async Task Drag(int idx, Vector2 from, Vector2 to, int steps = 6)
        {
            for (int s = 1; s <= steps; s++)
            {
                var a = from.Lerp(to, (s - 1) / (float)steps); var b = from.Lerp(to, s / (float)steps);
                Input.ParseInputEvent(new InputEventScreenDrag { Index = idx, Position = b * px, Relative = (b - a) * px, ScreenRelative = (b - a) * px });
                await Frames(1);
            }
            await Frames();
        }

        // test models are not packed into exported apps; the iOS build script copies them into Models/
        await Import(ProjectSettings.GlobalizePath(_mobile ? "user://Models/humanoid.glb" : "res://test/humanoid.glb"));
        _retopoWin.Visible = false;   // opened by the import over the middle of the view, where the joints below are
        _rigPreset.Select(RigPreset.List().IndexOf("humanoid"));
        await OnFitPressed();
        await Frames(5);
        GD.Print($"Touch test: viewport {GetViewport().GetVisibleRect().Size}, pick radius {_gizmo.PickRadius}");

        // 1. one finger on empty space (top-left sky) orbits
        float yaw0 = _orbit.Yaw;
        var p0 = new Vector2(60, _toolbar.GetGlobalRect().End.Y + 60);   // empty space below the toolbar, left of the model
        await Touch(0, p0, true);
        await Drag(0, p0, p0 + new Vector2(150, 0));
        await Touch(0, p0 + new Vector2(150, 0), false);
        Check(MathF.Abs(_orbit.Yaw - yaw0) > 0.5f, $"one-finger drag on empty space orbits (yaw {yaw0:F2} -> {_orbit.Yaw:F2})");

        // 2. one finger on a joint moves the joint, not the camera
        int hand = _p.Rig.Find("LeftHand");
        var head0 = _p.Rig.Bones[hand].Head;
        var hp = _orbit.Cam.UnprojectPosition(head0);
        float yaw1 = _orbit.Yaw;
        await Touch(0, hp, true);
        await Drag(0, hp, hp + new Vector2(0, -40));
        await Touch(0, hp + new Vector2(0, -40), false);
        var moved = _p.Rig.Bones[hand].Head.DistanceTo(head0);
        Check(moved > 0.01f && MathF.Abs(_orbit.Yaw - yaw1) < 1e-4f, $"one-finger drag on a joint moves it {moved:F3} m, camera yaw unchanged ({yaw1:F3} -> {_orbit.Yaw:F3})");

        // 3. two-finger spread zooms in; no orbit while two fingers are down
        float d0 = _orbit.Distance, yaw2 = _orbit.Yaw;
        Vector2 a = new(300, 400), b = new(500, 400);
        await Touch(0, a, true);
        await Touch(1, b, true);
        await Drag(1, b, b + new Vector2(200, 0));
        await Touch(1, b + new Vector2(200, 0), false);
        await Touch(0, a, false);
        Check(_orbit.Distance < d0 * 0.8f, $"two-finger spread zooms in (distance {d0:F2} -> {_orbit.Distance:F2})");
        Check(MathF.Abs(_orbit.Yaw - yaw2) < 1e-4f, $"no orbit during pinch (yaw {yaw2:F3} -> {_orbit.Yaw:F3})");
        Check(OrbitCamera.TouchCount == 0 && !OrbitCamera.MultiTouch, $"all fingers released (count {OrbitCamera.TouchCount})");

        // 4. a finger on the side panel does not orbit
        float yaw3 = _orbit.Yaw;
        var vp = GetViewport().GetVisibleRect().Size;
        var pp = new Vector2(vp.X - 120, _panel.GlobalPosition.Y + 14);   // over the "Steps" caption at the top of the panel (nothing to toggle)
        await Touch(0, pp, true);
        await Drag(0, pp, pp + new Vector2(-150, 0));
        await Touch(0, pp + new Vector2(-150, 0), false);
        Check(MathF.Abs(_orbit.Yaw - yaw3) < 1e-4f, $"drag over the panel does not orbit (yaw {yaw3:F3} -> {_orbit.Yaw:F3})");
        Check(_p.Rig.Bones[hand].Deform, "LeftHand still deforms (no control toggled by the test)");

        // 5. a drag on the view cube orbits (as if turning the cube), a tap on it snaps to a face
        var cr = _viewCube.GetGlobalRect();
        var cc = cr.GetCenter();
        float yaw4 = _orbit.Yaw;
        await Touch(0, cc, true);
        await Drag(0, cc, cc + new Vector2(60, 0));
        await Touch(0, cc + new Vector2(60, 0), false);
        Check(MathF.Abs(_orbit.Yaw - yaw4) > 0.5f, $"drag on the view cube orbits (yaw {yaw4:F2} -> {_orbit.Yaw:F2})");
        await Touch(0, cc, true);
        await Touch(0, cc, false);
        await Frames(30);   // the snap animates over 0.25 s
        static float Off(float a) { float r = Mathf.PosMod(a, MathF.PI / 2); return MathF.Min(r, MathF.PI / 2 - r); }
        bool axis = (MathF.Abs(_orbit.Pitch) < 1e-3f && Off(_orbit.Yaw) < 1e-3f) || MathF.Abs(MathF.Abs(_orbit.Pitch) - 1.5f) < 1e-3f;
        Check(axis, $"tap on the view cube snaps to a face view (yaw {_orbit.Yaw:F3}, pitch {_orbit.Pitch:F3})");

        await Shot(ProjectSettings.GlobalizePath("user://touch_test.png"));
        GD.Print("Touch test: " + (ok ? "all passed" : "FAILED"));
        GetTree().Quit(ok ? 0 : 1);
    }

    async Task Shot(string path)
    {
        for (int f = 0; f < 5; f++) await ToSignal(RenderingServer.Singleton, RenderingServer.SignalName.FramePostDraw);
        GetViewport().GetTexture().GetImage().SavePng(path);
    }

    // ------------------------------------------------------------------ world

    void BuildWorld()
    {
        var env = new Godot.Environment
        {
            BackgroundMode = Godot.Environment.BGMode.Color, BackgroundColor = new Color(0.15f, 0.16f, 0.19f),
            // a dim grey world, like Blender's default, so faces turned away from the light aren't black
            AmbientLightSource = Godot.Environment.AmbientSource.Color, AmbientLightColor = new Color(0.25f, 0.25f, 0.25f),
        };
        AddChild(new WorldEnvironment { Environment = env });
        // one light source, from the front, upper right
        AddChild(new DirectionalLight3D { RotationDegrees = new Vector3(-40, 30, 0), ShadowEnabled = false });
        _orbit = new OrbitCamera();
        AddChild(_orbit);
        _gizmo = new BoneGizmo();
        if (_touchUi) _gizmo.PickRadius = 28;   // fingertip-sized (logical pixels, after UI scale)
        AddChild(_gizmo);
        _gizmo.Cam = null;
        _gizmo.SelectionChanged += b => { if (b >= 0 && b < _boneList.ItemCount) _boneList.Select(b); OnBoneSelected(b); };
        _gizmo.Edited += () => { InvalidateWeights(); };
        _gizmo.EditBlocked += () => Log("Skeleton is posed: press 'Reset pose' to edit joints.");
    }

    // ------------------------------------------------------------------ UI

    const float PanelWidth = 420;
    Control _toolbar, _overlay;
    PanelContainer _panel;
    Label _overlayTitle, _overlayDetail;
    Window _console;
    ConfigFile _uiCfg;
    const string UiCfgPath = "user://ui.cfg";
    double _busyT;

    void BuildUi()
    {
        _uiCfg = new ConfigFile();
        _uiCfg.Load(UiCfgPath);   // missing on first run: every section starts open
        var layer = new CanvasLayer();
        AddChild(layer);

        // ---- toolbar (top-left corner): icon buttons, with View and Settings menus
        var bar = new PanelContainer();
        bar.AddThemeStyleboxOverride("panel", new StyleBoxFlat
        {
            BgColor = new Color(0.12f, 0.13f, 0.15f), CornerRadiusBottomRight = 12,
            ContentMarginLeft = 8, ContentMarginRight = 8, ContentMarginTop = 6, ContentMarginBottom = 6,
        });
        layer.AddChild(bar);
        _toolbar = bar;
        var tools = new HBoxContainer();
        tools.AddThemeConstantOverride("separation", 4);
        bar.AddChild(tools);
        ToolIcon(tools, "file-up", "Import a .glb or .gltf (replaces the current model)", OnImportPressed);
        _btnRetopoWin = ToolIcon(tools, "grid-3x3", "Retopology (opens after every import)", () => _retopoWin.Visible = true);
        _btnExport = ToolIcon(tools, _touchUi ? "share" : "download",
            "Export .glb. Needs a retopologized mesh; with a fitted skeleton the export is rigged (weights are computed if needed), otherwise it is a static mesh with UVs.",
            OnExportPressed);
        tools.AddChild(new VSeparator());

        var view = ToolMenu(tools, "eye", "Show");
        _showSource = new MenuToggle(view, "Source model", true, v => { if (_sourceNode != null) _sourceNode.Visible = v; });
        _showRetopo = new MenuToggle(view, "Retopologized mesh", true, _ => RefreshVisibility());
        _wire = new MenuToggle(view, "Wireframe", true, DisplayMesh.SetWire);
        _showSkel = new MenuToggle(view, "Skeleton", true, v => _gizmo.Visible = v);

        var settings = ToolMenu(tools, "settings", "Settings");
        _faceNegZ = new MenuToggle(settings, "Model faces −Z (rotate 180° on import)", false,
            v => { if (_emptyFaceNegZ != null && _emptyFaceNegZ.ButtonPressed != v) _emptyFaceNegZ.ButtonPressed = v; });
        settings.AddSeparator("UV padding for texture size");
        int firstTex = settings.ItemCount;
        foreach (int t in new[] { 512, 1024, 2048, 4096 })
        {
            settings.AddRadioCheckItem($"{t} px");
            settings.SetItemChecked(settings.ItemCount - 1, t == _texSizeValue);
        }
        settings.IndexPressed += i =>
        {
            int k = (int)i - firstTex;
            if (k < 0 || k > 3) return;
            _texSizeValue = 512 << k;
            for (int j = 0; j < 4; j++) settings.SetItemChecked(firstTex + j, j == k);
            Log($"UV padding for {_texSizeValue} px textures");
        };
        ToolIcon(tools, "terminal", "Console", ToggleConsole);

        // ---- top right of the 3D view: frame + orientation cube
        var corner = new HBoxContainer
        {
            AnchorLeft = 1, AnchorRight = 1, OffsetRight = -PanelWidth - 12, OffsetLeft = -PanelWidth - 12 - 170, OffsetTop = 10,
            Alignment = BoxContainer.AlignmentMode.End, MouseFilter = Control.MouseFilterEnum.Ignore,
        };
        layer.AddChild(corner);
        _viewTools = corner;
        var frame = IconOnly("scan", "Frame the model");
        frame.Flat = true;
        frame.SizeFlagsVertical = Control.SizeFlags.ShrinkBegin;
        frame.Pressed += () => { if (_p.Source != null) _orbit.Frame(_p.Source.Mesh.Bounds()); };
        corner.AddChild(frame);
        _viewCube = new ViewCube { Orbit = _orbit };
        corner.AddChild(_viewCube);

        // ---- status line (bottom left)
        var statusBox = new PanelContainer
        {
            AnchorTop = 1, AnchorBottom = 1, OffsetLeft = 10, OffsetTop = -44, OffsetBottom = -10, MouseFilter = Control.MouseFilterEnum.Ignore,
        };
        statusBox.AddThemeStyleboxOverride("panel", new StyleBoxFlat
        {
            BgColor = new Color(0.12f, 0.13f, 0.15f, 0.85f), CornerRadiusTopLeft = 8, CornerRadiusTopRight = 8,
            CornerRadiusBottomLeft = 8, CornerRadiusBottomRight = 8, ContentMarginLeft = 10, ContentMarginRight = 10,
            ContentMarginTop = 4, ContentMarginBottom = 4,
        });
        layer.AddChild(statusBox);
        _statusBox = statusBox;
        _status = new Label { Text = "Idle", Modulate = new Color(1, 1, 1, 0.8f) };
        statusBox.AddChild(_status);

        // ---- side panel: rigging, each part a collapsible section
        var panel = new PanelContainer
        {
            AnchorLeft = 1, AnchorRight = 1, AnchorBottom = 1, OffsetLeft = -PanelWidth,
        };
        _panel = panel;
        layer.AddChild(panel);
        panel.AddThemeStyleboxOverride("panel", new StyleBoxFlat { BgColor = new Color(0.12f, 0.13f, 0.15f) });
        var scroll = new ScrollContainer { HorizontalScrollMode = ScrollContainer.ScrollMode.Disabled };
        panel.AddChild(scroll);
        var margin = new MarginContainer { SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        foreach (var side in new[] { "left", "right", "top", "bottom" }) margin.AddThemeConstantOverride("margin_" + side, 8);
        scroll.AddChild(margin);
        var vb = new VBoxContainer { SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        vb.AddThemeConstantOverride("separation", 6);
        margin.AddChild(vb);
        var steps = new Label
        {
            Text = "Rigging is optional: without a skeleton the export is a static mesh with UVs.",
            Modulate = new Color(1, 1, 1, 0.6f), AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(300, 0),
        };
        steps.AddThemeFontSizeOverride("font_size", 13);
        vb.AddChild(steps);

        var sk = Section(vb, "Skeleton & weights", "bone");
        _rigPreset = Option(sk, "Rig preset", RigPreset.List().ToArray());
        var fr = new HBoxContainer();
        sk.AddChild(fr);
        _btnFit = Btn(fr, "Fit skeleton", () => _ = OnFitPressed(), "wand-sparkles");
        _btnLoadPreset = Btn(fr, "Load preset…", OnLoadPresetPressed, "folder-open");
        _btnCenter = Btn(sk, "Center joints in volume", () => _ = OnCenterPressed(), "crosshair");
        _mirror = Check(sk, "Mirror X edits", true, v => _gizmo.Mirror = v);
        _boneList = Option(sk, "Bone", Array.Empty<string>());
        _boneList.ItemSelected += i => { _gizmo.Select((int)i); OnBoneSelected((int)i); };
        var eb = new HBoxContainer();
        sk.AddChild(eb);
        _btnAddBone = Btn(eb, "Add child", OnAddBone, "plus");
        _btnDelBone = Btn(eb, "Delete", OnDeleteBone, "trash-2");
        _btnSym = Btn(eb, "Symmetrize", OnSymmetrize, "flip-horizontal-2");
        _btnSym.TooltipText = "Copy every Left* bone onto its Right* partner (mirrored), creating missing partners";
        _btnAddBone.TooltipText = "Add a bone under the selected one";
        if (_touchUi) Hint(sk, "Symmetrize copies every Left* bone onto its Right* partner, creating missing ones.");
        _btnDelBone.TooltipText = "Delete the selected bone; its children move to its parent";
        var rn = new HBoxContainer();
        sk.AddChild(rn);
        _renameEdit = new LineEdit { PlaceholderText = "bone name", SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        rn.AddChild(_renameEdit);
        _btnRename = Btn(rn, "Rename", OnRenameBone, "pencil");
        _btnRename.CustomMinimumSize = new Vector2(100, 0);
        _btnRename.SizeFlagsHorizontal = Control.SizeFlags.ShrinkEnd;
        _parentList = Option(sk, "Parent", Array.Empty<string>());
        _parentList.ItemSelected += i => OnReparent((int)i);
        _deformCheck = Check(sk, "Deforms mesh (gets weights)", true, OnDeformToggled);
        var sp = new HBoxContainer();
        sk.AddChild(sp);
        _presetName = new LineEdit { PlaceholderText = "preset name", SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        sp.AddChild(_presetName);
        _btnSavePreset = Btn(sp, "Save", OnSavePreset, "save");
        _btnSavePreset.TooltipText = "Save this skeleton as a reusable preset (joint positions relative to the model's bounds)";
        _btnSavePreset.CustomMinimumSize = new Vector2(100, 0);
        _btnSavePreset.SizeFlagsHorizontal = Control.SizeFlags.ShrinkEnd;
        sk.AddChild(new HSeparator());
        _falloff = Spin(sk, "Weight falloff exponent", 1, 8, 3, 0.5);
        _btnWeights = Btn(sk, "Compute weights", () => _ = OnWeightsPressed(), "paintbrush");
        _btnWeights.TooltipText = "Optional: Export computes them too when a skeleton is fitted";
        _heat = Check(sk, "Show selected bone's weights", false, _ => UpdateHeatmap());

        var spz = Section(vb, "Preview pose", "person-standing");
        _rx = Slider(spz, "Rotate X", OnPoseSlider);
        _ry = Slider(spz, "Rotate Y", OnPoseSlider);
        _rz = Slider(spz, "Rotate Z", OnPoseSlider);
        var hb = new HBoxContainer();
        spz.AddChild(hb);
        _btnTestPose = Btn(hb, "Test pose", () => Log(ApplyNextTestPose()), "play");
        _btnTestPose.TooltipText = "Humanoid: a standard bend. Rigs from a preset with named poses: cycles through them.";
        _btnResetPose = Btn(hb, "Reset pose", ResetPose, "rotate-ccw");

        BuildRetopoWindow(layer);
        BuildEmptyState(layer);
        BuildConsole();
        BuildOverlay();
        UpdateVoxelInfo();
        void Inset() { _orbit.RightInset = MathF.Min(0.6f, PanelWidth / MathF.Max(1, GetViewport().GetVisibleRect().Size.X)); _orbit.Refresh(); }
        GetViewport().SizeChanged += Inset;
        Callable.From(Inset).CallDeferred();
        if (_touchUi)
        {
            EnlargeForTouch(vb);
            // keep clear of rounded corners / home indicator. The safe area and window size are both in pixels;
            // insets that come out negative or implausibly large mean the units disagree, so they are ignored.
            var safe = DisplayServer.GetDisplaySafeArea();
            var win = DisplayServer.WindowGetSize();
            float sc = GetWindow().ContentScaleFactor;
            Log($"Display: window {win}, safe area {safe}, UI scale {sc}");
            if (_mobile && safe.Size.X > 0)
            {
                float top = safe.Position.Y, bottom = win.Y - safe.End.Y, right = win.X - safe.End.X, left = safe.Position.X;
                bool plausible = top >= 0 && bottom >= 0 && right >= 0 && left >= 0 && top < 0.15f * win.Y && bottom < 0.15f * win.Y
                                 && right < 0.15f * win.X && left < 0.15f * win.X;
                if (plausible)
                {
                    panel.OffsetTop = top / sc;
                    panel.OffsetBottom = -bottom / sc;
                    panel.OffsetRight = -right / sc;
                    panel.OffsetLeft = panel.OffsetRight - PanelWidth;
                    bar.OffsetTop = top / sc;
                    bar.OffsetLeft = left / sc;
                    corner.OffsetTop = top / sc + 10;
                    corner.OffsetRight = panel.OffsetLeft - 12;
                    corner.OffsetLeft = corner.OffsetRight - 170;
                    statusBox.OffsetTop = -bottom / sc - 44;
                    statusBox.OffsetBottom = -bottom / sc - 10;
                    statusBox.OffsetLeft = left / sc + 10;
                }
            }
        }
        UpdateChrome();
    }

    PanelContainer _retopoWin;
    Button _btnRetopoWin;
    Control _empty;
    CheckBox _emptyFaceNegZ;

    /// <summary>
    /// Remesh settings in a floating panel (drag it by its title), opened after every import so the next step is
    /// right there. A panel rather than a window, so the 3D view around it still orbits and zooms.
    /// </summary>
    void BuildRetopoWindow(CanvasLayer layer)
    {
        var card = new PanelContainer
        {
            AnchorLeft = 0.5f, AnchorRight = 0.5f, AnchorTop = 0.5f, AnchorBottom = 0.5f,
            GrowHorizontal = Control.GrowDirection.Both, GrowVertical = Control.GrowDirection.Both, Visible = false,
        };
        card.AddThemeStyleboxOverride("panel", new StyleBoxFlat
        {
            BgColor = new Color(0.12f, 0.13f, 0.15f), CornerRadiusTopLeft = 10, CornerRadiusTopRight = 10,
            CornerRadiusBottomLeft = 10, CornerRadiusBottomRight = 10, BorderColor = new Color(1, 1, 1, 0.08f),
            BorderWidthLeft = 1, BorderWidthRight = 1, BorderWidthTop = 1, BorderWidthBottom = 1,
            ShadowColor = new Color(0, 0, 0, 0.45f), ShadowSize = 18,
            ContentMarginLeft = 16, ContentMarginRight = 16, ContentMarginTop = 10, ContentMarginBottom = 14,
        });
        layer.AddChild(card);
        _retopoWin = card;
        var v = new VBoxContainer { CustomMinimumSize = new Vector2(440, 0) };
        v.AddThemeConstantOverride("separation", 8);
        card.AddChild(v);
        var head = new HBoxContainer { MouseDefaultCursorShape = Control.CursorShape.Move };
        v.AddChild(head);
        var icon = new TextureRect
        {
            Texture = Icons.Get("grid-3x3"), ExpandMode = TextureRect.ExpandModeEnum.IgnoreSize,
            StretchMode = TextureRect.StretchModeEnum.KeepAspectCentered, CustomMinimumSize = new Vector2(20, 20),
            MouseFilter = Control.MouseFilterEnum.Ignore,
        };
        head.AddChild(icon);
        var title = new Label { Text = "Retopology", SizeFlagsHorizontal = Control.SizeFlags.ExpandFill, MouseFilter = Control.MouseFilterEnum.Ignore };
        title.AddThemeFontSizeOverride("font_size", 17);
        head.AddChild(title);
        var close = IconOnly("x", "Close");
        close.Flat = true;
        close.Pressed += () => card.Visible = false;
        head.AddChild(close);
        // drag by the title row
        head.MouseFilter = Control.MouseFilterEnum.Stop;
        head.GuiInput += e =>
        {
            if (e is InputEventMouseMotion mm && (mm.ButtonMask & MouseButtonMask.Left) != 0)
            {
                card.OffsetLeft += mm.Relative.X; card.OffsetRight += mm.Relative.X;
                card.OffsetTop += mm.Relative.Y; card.OffsetBottom += mm.Relative.Y;
            }
        };
        v.AddChild(new HSeparator());
        _retopoMode = Option(v, "Mode", new[] { "Quad (recommended)", "Voxel + decimate" });
        _retopoMode.ItemSelected += _ => UpdateVoxelInfo();
        _spacing = Option(v, "Spacing (quad mode)", new[] { "Auto (recommended)", "Uniform", "Adaptive" });
        _spacing.TooltipText = "Adaptive spends more quads where the surface curves. Auto builds both and keeps the closer match.";
        if (_touchUi) Hint(v, "Auto builds uniform and adaptive spacing and keeps the closer match.");
        _tris = Spin(v, "Target triangles", 200, 200000, 8000, 100, _ => UpdateVoxelInfo());
        _voxel = Spin(v, "Voxel resolution (voxel mode)", 64, 512, 256, 16, _ => UpdateVoxelInfo());
        _voxelInfo = new Label { Modulate = new Color(1, 1, 1, 0.7f), AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(300, 0) };
        v.AddChild(_voxelInfo);
        _close = Spin(v, "Gap closing (voxels)", 0, 3, 1, 1);
        _relax = Spin(v, "Relax iterations", 0, 10, 4, 1);
        _triInfo = new Label { AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(300, 0) };
        v.AddChild(_triInfo);
        var row = new HBoxContainer();
        row.AddThemeConstantOverride("separation", 8);
        v.AddChild(row);
        var later = IconButton("Later", "x");
        later.SizeFlagsHorizontal = Control.SizeFlags.ExpandFill;
        later.Pressed += () => _retopoWin.Visible = false;
        row.AddChild(later);
        _btnRetopo = Btn(row, "Retopologize", () => _ = OnRetopoPressed(), "grid-3x3");
        if (_touchUi) EnlargeForTouch(v);
    }

    /// <summary>Before anything is imported: nothing but a big import button (and the one import option).</summary>
    void BuildEmptyState(CanvasLayer layer)
    {
        var center = new CenterContainer { AnchorRight = 1, AnchorBottom = 1 };
        layer.AddChild(center);
        _empty = center;
        var v = new VBoxContainer();
        v.AddThemeConstantOverride("separation", 14);
        center.AddChild(v);
        var big = new Button
        {
            Text = "Import Mesh", Icon = Icons.Get("file-up", 64), CustomMinimumSize = new Vector2(340, 96),
            SizeFlagsHorizontal = Control.SizeFlags.ShrinkCenter,
        };
        big.AddThemeFontSizeOverride("font_size", 26);
        big.AddThemeConstantOverride("icon_max_width", 32);
        big.AddThemeConstantOverride("h_separation", 12);
        big.Pressed += OnImportPressed;
        v.AddChild(big);
        _emptyFaceNegZ = new CheckBox { Text = "Model faces −Z (rotate 180° on import)", SizeFlagsHorizontal = Control.SizeFlags.ShrinkCenter };
        _emptyFaceNegZ.Toggled += on => _faceNegZ.ButtonPressed = on;
        v.AddChild(_emptyFaceNegZ);
        var hint = new Label
        {
            Text = _touchUi ? "Opens .glb / .gltf from Files › On My iPad › ModelMate › Models, or share a file to ModelMate."
                            : "A .glb or .gltf: a dense sculpt or a generated mesh.",
            HorizontalAlignment = HorizontalAlignment.Center, Modulate = new Color(1, 1, 1, 0.6f),
            AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(420, 0),
            SizeFlagsHorizontal = Control.SizeFlags.ShrinkCenter,
        };
        v.AddChild(hint);
        if (_touchUi) EnlargeForTouch(v);
    }

    /// <summary>The empty state until a model is loaded, then the toolbar and panel.</summary>
    void UpdateChrome()
    {
        bool src = _p.Source != null;
        _empty.Visible = !src;
        _toolbar.Visible = src;
        _panel.Visible = src;
        _viewTools.Visible = src;
        _statusBox.Visible = src;
    }

    /// <summary>A button with a Lucide icon (sized for the UI, sharp at any scale).</summary>
    Button IconButton(string text, string icon)
    {
        var b = new Button { Text = text, Icon = Icons.Get(icon) };
        b.AddThemeConstantOverride("icon_max_width", 18);
        return b;
    }

    Control _viewTools, _statusBox;
    ViewCube _viewCube;

    /// <summary>Icon-only button (tooltip for the name), touch-sized on touch screens.</summary>
    Button IconOnly(string icon, string tip)
    {
        var b = new Button { Icon = Icons.Get(icon, 56), TooltipText = tip, CustomMinimumSize = new Vector2(44, 44) };
        b.AddThemeConstantOverride("icon_max_width", 24);
        b.IconAlignment = HorizontalAlignment.Center;
        return b;
    }

    Button ToolIcon(Container c, string icon, string tip, Action onPress)
    {
        var b = IconOnly(icon, tip);
        b.Flat = true;
        b.Pressed += onPress;
        c.AddChild(b);
        _actionButtons.Add(b);
        return b;
    }

    /// <summary>Icon button that opens a menu of options; returns the menu.</summary>
    PopupMenu ToolMenu(Container c, string icon, string tip)
    {
        var mb = new MenuButton { Icon = Icons.Get(icon, 56), TooltipText = tip, CustomMinimumSize = new Vector2(44, 44), Flat = true };
        mb.AddThemeConstantOverride("icon_max_width", 24);
        mb.IconAlignment = HorizontalAlignment.Center;
        c.AddChild(mb);
        var pm = mb.GetPopup();
        pm.HideOnCheckableItemSelection = false;
        if (_touchUi) pm.AddThemeConstantOverride("v_separation", 18);
        return pm;
    }

    /// <summary>A checkable menu item with the same ButtonPressed API as a CheckBox.</summary>
    sealed class MenuToggle
    {
        readonly PopupMenu _m;
        readonly int _i;
        readonly Action<bool> _changed;

        public MenuToggle(PopupMenu m, string text, bool on, Action<bool> changed)
        {
            _m = m; _changed = changed;
            m.AddCheckItem(text);
            _i = m.ItemCount - 1;
            m.SetItemChecked(_i, on);
            m.IndexPressed += i => { if (i == _i) ButtonPressed = !ButtonPressed; };
        }

        public bool ButtonPressed
        {
            get => _m.IsItemChecked(_i);
            set { if (value == ButtonPressed) return; _m.SetItemChecked(_i, value); _changed?.Invoke(value); }
        }
    }

    /// <summary>
    /// A collapsible section: a header that toggles the body below it. Open/closed is remembered across launches.
    /// </summary>
    VBoxContainer Section(Container c, string title, string icon)
    {
        bool open = (bool)_uiCfg.GetValue("sections", title, true);
        c.AddChild(new HSeparator());
        var row = new HBoxContainer();
        c.AddChild(row);
        var header = new Button
        {
            Text = title, ToggleMode = true, ButtonPressed = open, Flat = true, Alignment = HorizontalAlignment.Left,
            SizeFlagsHorizontal = Control.SizeFlags.ExpandFill, Icon = Icons.Get(icon),
        };
        header.AddThemeFontSizeOverride("font_size", 15);
        header.AddThemeConstantOverride("icon_max_width", 18);
        row.AddChild(header);
        var chevron = new TextureRect
        {
            Texture = Icons.Get(open ? "chevron-down" : "chevron-right"), ExpandMode = TextureRect.ExpandModeEnum.IgnoreSize,
            StretchMode = TextureRect.StretchModeEnum.KeepAspectCentered, CustomMinimumSize = new Vector2(18, 18),
            MouseFilter = Control.MouseFilterEnum.Ignore, Modulate = new Color(1, 1, 1, 0.7f),
        };
        row.AddChild(chevron);
        var body = new VBoxContainer { Visible = open, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        body.AddThemeConstantOverride("separation", 6);
        c.AddChild(body);
        header.Toggled += on =>
        {
            body.Visible = on;
            chevron.Texture = Icons.Get(on ? "chevron-down" : "chevron-right");
            _uiCfg.SetValue("sections", title, on);
            _uiCfg.Save(UiCfgPath);
        };
        return body;
    }

    /// <summary>The log, in its own window (toolbar: Console). It keeps collecting while closed.</summary>
    void BuildConsole()
    {
        _console = new Window { Title = "Console", Size = new Vector2I(760, 460), MinSize = new Vector2I(360, 200), Visible = false };
        _console.CloseRequested += () => _console.Hide();
        AddChild(_console);
        var bg = new Panel { AnchorRight = 1, AnchorBottom = 1 };
        bg.AddThemeStyleboxOverride("panel", new StyleBoxFlat { BgColor = new Color(0.1f, 0.11f, 0.13f) });
        _console.AddChild(bg);
        var m = new MarginContainer { AnchorRight = 1, AnchorBottom = 1 };
        foreach (var side in new[] { "left", "right", "top", "bottom" }) m.AddThemeConstantOverride("margin_" + side, 8);
        _console.AddChild(m);
        var v = new VBoxContainer();
        m.AddChild(v);
        var tools = new HBoxContainer();
        v.AddChild(tools);
        var copy = IconButton("Copy all", "copy");
        copy.Pressed += () => DisplayServer.ClipboardSet(_log.GetParsedText());
        tools.AddChild(copy);
        var clear = IconButton("Clear", "eraser");
        clear.Pressed += () => _log.Clear();
        tools.AddChild(clear);
        _log = new RichTextLabel
        {
            ScrollFollowing = true, SelectionEnabled = true, SizeFlagsVertical = Control.SizeFlags.ExpandFill,
            SizeFlagsHorizontal = Control.SizeFlags.ExpandFill,
        };
        _log.AddThemeFontSizeOverride("normal_font_size", _touchUi ? 13 : 11);
        v.AddChild(_log);
        if (_touchUi) EnlargeForTouch(tools);
    }

    void ToggleConsole()
    {
        if (_console.Visible) _console.Hide();
        else _console.PopupCentered();
    }

    /// <summary>Dims the screen and blocks input while a step runs, with the step's title and a busy bar.</summary>
    void BuildOverlay()
    {
        var layer = new CanvasLayer { Layer = 10 };
        AddChild(layer);
        var dim = new ColorRect
        {
            Color = new Color(0, 0, 0, 0.55f), AnchorRight = 1, AnchorBottom = 1, MouseFilter = Control.MouseFilterEnum.Stop,
            Visible = false,
        };
        layer.AddChild(dim);
        _overlay = dim;
        var center = new CenterContainer { AnchorRight = 1, AnchorBottom = 1, MouseFilter = Control.MouseFilterEnum.Ignore };
        dim.AddChild(center);
        var card = new PanelContainer();
        card.AddThemeStyleboxOverride("panel", new StyleBoxFlat
        {
            BgColor = new Color(0.12f, 0.13f, 0.15f), CornerRadiusTopLeft = 10, CornerRadiusTopRight = 10,
            CornerRadiusBottomLeft = 10, CornerRadiusBottomRight = 10,
            ContentMarginLeft = 24, ContentMarginRight = 24, ContentMarginTop = 18, ContentMarginBottom = 18,
        });
        center.AddChild(card);
        var v = new VBoxContainer();
        v.AddThemeConstantOverride("separation", 10);
        card.AddChild(v);
        _overlayTitle = new Label { HorizontalAlignment = HorizontalAlignment.Center };
        _overlayTitle.AddThemeFontSizeOverride("font_size", 20);
        v.AddChild(_overlayTitle);
        v.AddChild(new ProgressBar { Indeterminate = true, ShowPercentage = false, CustomMinimumSize = new Vector2(340, 10) });
        _overlayDetail = new Label { HorizontalAlignment = HorizontalAlignment.Center, Modulate = new Color(1, 1, 1, 0.65f) };
        _overlayDetail.AddThemeFontSizeOverride("font_size", 13);
        v.AddChild(_overlayDetail);
    }

    static readonly Dictionary<string, string> StepTitles = new()
    {
        ["import"] = "Importing model…", ["retopology"] = "Retopologizing…", ["fit skeleton"] = "Fitting skeleton…",
        ["center joints"] = "Centering joints…", ["weights"] = "Computing weights…", ["export"] = "Exporting…",
    };

    /// <summary>Shows what a running step is doing now (overlay title and status line); lets the UI repaint.</summary>
    async Task BusyStep(string title)
    {
        _overlayTitle.Text = title;
        _status.Text = "Working: " + title;
        await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
    }

    static void Hint(Container c, string text)
    {
        var l = new Label { Text = text, Modulate = new Color(1, 1, 1, 0.7f), AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(300, 0) };
        l.AddThemeFontSizeOverride("font_size", 13);
        c.AddChild(l);
    }

    Button Btn(Container c, string text, Action onPress, string icon = null)
    {
        var b = new Button { Text = text, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        if (icon != null) { b.Icon = Icons.Get(icon); b.AddThemeConstantOverride("icon_max_width", 18); }
        b.Pressed += onPress;
        c.AddChild(b);
        _actionButtons.Add(b);
        return b;
    }

    static CheckBox Check(Container c, string text, bool on, Action<bool> toggled)
    {
        var cb = new CheckBox { Text = text, ButtonPressed = on };
        cb.Toggled += v => toggled(v);
        c.AddChild(cb);
        return cb;
    }

    static SpinBox Spin(Container c, string label, double min, double max, double val, double step, Action<double> changed = null)
    {
        var hb = new HBoxContainer();
        hb.AddChild(new Label { Text = label, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill, ClipText = true });
        var sb = new SpinBox { MinValue = min, MaxValue = max, Value = val, Step = step, CustomMinimumSize = new Vector2(110, 0) };
        if (changed != null) sb.ValueChanged += v => changed(v);
        hb.AddChild(sb);
        c.AddChild(hb);
        return sb;
    }

    static OptionButton Option(Container c, string label, string[] items)
    {
        var hb = new HBoxContainer();
        hb.AddChild(new Label { Text = label, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill });
        var ob = new OptionButton { CustomMinimumSize = new Vector2(150, 0), ClipText = true };
        foreach (var i in items) ob.AddItem(i);
        hb.AddChild(ob);
        c.AddChild(hb);
        return ob;
    }

    static HSlider Slider(Container c, string label, Action onChange)
    {
        var hb = new HBoxContainer();
        hb.AddChild(new Label { Text = label, CustomMinimumSize = new Vector2(80, 0) });
        var s = new HSlider { MinValue = -180, MaxValue = 180, Step = 1, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        s.ValueChanged += _ => onChange();
        hb.AddChild(s);
        c.AddChild(hb);
        return s;
    }

    void Log(string s) => _logQ.Enqueue(s);

    public override void _Notification(int what)
    {
        // returning from another app (e.g. after "Copy to ModelMate"): pick up files iOS left in Documents/Inbox
        if (what == NotificationApplicationResumed || what == NotificationApplicationFocusIn) IosBridge.SweepInbox();
    }

    public override void _Process(double delta)
    {
        if (_log == null) return;
        while (IosBridge.Messages.TryDequeue(out var m)) Log(m);
        if (!_busy && IosBridge.Received.TryDequeue(out var received))
        {
            // several at once: keep the newest, the others are already saved in Models/
            while (IosBridge.Received.TryDequeue(out var newer)) received = newer;
            Log($"Received {System.IO.Path.GetFileName(received)} from another app (saved in Models/)");
            _ = Import(received);
        }
        while (_logQ.TryDequeue(out var s))
        {
            _log.AddText(s + "\n");
            GD.Print(s);
        }
        if (_gizmo.Cam == null && _orbit.Cam != null) _gizmo.Cam = _orbit.Cam;
        if (_busy)
        {
            _busyT += delta;
            _overlayDetail.Text = $"{_busyT:0} s" + (_mobile ? " · keep ModelMate open: switching apps pauses it" : "");
        }
        _memSampleT += delta;
        if (_memSampleT > 0.25) { _memSampleT = 0; SampleMemory(); }
        if (_mobile && OS.IsDebugBuild())
        {
            _cmdT += delta;
            if (_cmdT > 1) { _cmdT = 0; _ = PollDebugCommands(); }
        }
    }

    double _cmdT;
    bool _cmdBusy;

    async Task Frames(int n) { for (int i = 0; i < n; i++) await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame); }

    /// <summary>
    /// Debug builds on a device: commands dropped into user://cmd.txt (one per line) let a connected Mac inspect and
    /// drive the running app. "shot NAME" saves user://shots/NAME.png; "import PATH" imports a model (user:// path);
    /// "tap X Y" and "drag X0 Y0 X1 Y1" send real touch events at window pixels (the same pixels as a screenshot);
    /// "wait S" pauses; "ui-smoke IN OUT" runs the UI smoke test; "share PATH" opens the share
    /// sheet for a file; "quit" exits. The file is deleted once read.
    /// </summary>
    async Task PollDebugCommands()
    {
        const string f = "user://cmd.txt";
        if (_cmdBusy || !Godot.FileAccess.FileExists(f)) return;
        _cmdBusy = true;
        try
        {
            var lines = Godot.FileAccess.GetFileAsString(f).Split('\n', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
            DirAccess.RemoveAbsolute(ProjectSettings.GlobalizePath(f));
            foreach (var line in lines)
            {
                var a = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                GD.Print($"[cmd] {line}");
                switch (a[0])
                {
                    case "shot":
                        DirAccess.MakeDirRecursiveAbsolute(ProjectSettings.GlobalizePath("user://shots"));
                        await Shot(ProjectSettings.GlobalizePath($"user://shots/{(a.Length > 1 ? a[1] : "shot")}.png"));
                        break;
                    case "share" when a.Length >= 2:
                    {
                        var r = _btnExport.GetGlobalRect();
                        bool shown = IosBridge.Share(ProjectSettings.GlobalizePath(a[1]), r.Position.X, r.Position.Y, r.Size.X, r.Size.Y);
                        GD.Print($"[cmd] share sheet {(shown ? "presented" : "NOT presented")}");
                        break;
                    }
                    case "import" when a.Length >= 2:
                        await Import(ProjectSettings.GlobalizePath(a[1]));
                        break;
                    case "wait" when a.Length >= 2:
                        await ToSignal(GetTree().CreateTimer(double.Parse(a[1], System.Globalization.CultureInfo.InvariantCulture)), SceneTreeTimer.SignalName.Timeout);
                        break;
                    case "tap" when a.Length >= 3:
                    {
                        var p = new Vector2(float.Parse(a[1], System.Globalization.CultureInfo.InvariantCulture), float.Parse(a[2], System.Globalization.CultureInfo.InvariantCulture));
                        Input.ParseInputEvent(new InputEventScreenTouch { Index = 0, Position = p, Pressed = true });
                        await Frames(3);
                        Input.ParseInputEvent(new InputEventScreenTouch { Index = 0, Position = p, Pressed = false });
                        await Frames(10);
                        break;
                    }
                    case "drag" when a.Length >= 5:
                    {
                        float F(int i) => float.Parse(a[i], System.Globalization.CultureInfo.InvariantCulture);
                        Vector2 p0 = new(F(1), F(2)), p1 = new(F(3), F(4));
                        Input.ParseInputEvent(new InputEventScreenTouch { Index = 0, Position = p0, Pressed = true });
                        await Frames(3);
                        for (int k = 1; k <= 12; k++)
                        {
                            var q0 = p0.Lerp(p1, (k - 1) / 12f); var q1 = p0.Lerp(p1, k / 12f);
                            Input.ParseInputEvent(new InputEventScreenDrag { Index = 0, Position = q1, Relative = q1 - q0, ScreenRelative = q1 - q0 });
                            await Frames(1);
                        }
                        Input.ParseInputEvent(new InputEventScreenTouch { Index = 0, Position = p1, Pressed = false });
                        await Frames(10);
                        break;
                    }
                    case "ui-smoke" when a.Length >= 3:
                        await UiSmoke(a[1], a[2]);
                        break;
                    case "quit":
                        GetTree().Quit();
                        break;
                }
            }
        }
        catch (Exception e) { GD.PrintErr("[cmd] " + e.Message); }
        finally { _cmdBusy = false; }
    }

    void UpdateVoxelInfo()
    {
        if (_p.Source == null) { _voxelInfo.Text = "Voxel size: (import first)"; return; }
        var s = _p.Source.Mesh.Bounds().Size;
        float ext = MathF.Max(s.X, MathF.Max(s.Y, s.Z));
        if (_retopoMode.Selected == 0)
        {
            // source area over-counts internal faces, so this is a lower bound on the final spacing
            float rho = MathF.Sqrt((float)(Pipeline.Area(_p.Source.Mesh) / Math.Max(1, (int)_tris.Value / 2)));
            _voxelInfo.Text = $"Quad size ≈ {rho:0.####} units or more  (features thinner than that may be lost)";
        }
        else
        {
            float h = ext / (float)_voxel.Value;
            _voxelInfo.Text = $"Voxel size: {h:0.####} units  (features thinner than ~{2 * h:0.###} may be lost)";
        }
    }

    void UpdateButtons()
    {
        bool src = _p.Source != null, retopo = _p.Welded != null, rig = _p.Rig != null, w = _p.WeldedBones != null && !_weightsStale;
        foreach (var b in _actionButtons) b.Disabled = _busy;
        if (_busy) return;
        bool sel = rig && _gizmo.Selected >= 0 && _gizmo.Selected < _p.Rig.Bones.Count;
        _btnFit.Disabled = !src;
        _btnLoadPreset.Disabled = !src;
        _btnCenter.Disabled = !rig;
        _btnAddBone.Disabled = !sel;
        _btnDelBone.Disabled = !sel || _gizmo.Selected == 0;
        _btnRename.Disabled = !sel;
        _btnSym.Disabled = !rig;
        _btnSavePreset.Disabled = !rig;
        _btnRetopo.Disabled = !src;
        _btnRetopoWin.Disabled = !src;
        _btnWeights.Disabled = !rig || !retopo;
        _btnTestPose.Disabled = !w;
        _btnResetPose.Disabled = !w;
        _btnExport.Disabled = !retopo;   // only retopology is required; a skeleton makes the export rigged
    }

    async Task RunBusy(string what, Func<Task> work)
    {
        if (_busy) return;
        _busy = true;
        _overlayTitle.Text = StepTitles.TryGetValue(what, out var title) ? title : what + "…";
        _status.Text = "Working: " + _overlayTitle.Text;
        _busyT = 0;
        _overlayDetail.Text = _mobile ? "Keep ModelMate open: switching apps pauses it" : "";
        _overlay.Visible = true;
        if (_mobile) DisplayServer.ScreenSetKeepOn(true);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        UpdateButtons();
        await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame); // let the label repaint
        await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
        try
        {
            await work();
            _status.Text = "Done: " + what;
        }
        catch (Exception e)
        {
            Log("ERROR: " + e.Message);
            GD.PrintErr(e.ToString());
            _status.Text = "Failed: " + what + " (see Console)";
            if (!_console.Visible) _console.PopupCentered();
        }
        finally
        {
            _busy = false;
            _overlay.Visible = false;
            if (_mobile) DisplayServer.ScreenSetKeepOn(false);
            SampleMemory();
            Log($"[{what}] {sw.Elapsed.TotalSeconds:0.0} s, {MemoryText()}");
            UpdateButtons();
        }
    }

    // ------------------------------------------------------------------ steps

    void OnImportPressed()
    {
        var fd = OpenDialog("Import glTF", "*.glb, *.gltf ; glTF models", "user://Models");
        fd.FileSelected += path => { _ = Import(ProjectSettings.GlobalizePath(path)); fd.QueueFree(); };
    }

    async Task Import(string path)
    {
        await RunBusy("import", () =>
        {
            ClearAll();
            _p.S.RotateY = _faceNegZ.ButtonPressed ? 180 : 0;
            _p.Import(path);
            _sourcePath = path;
            // shown from the imported mesh data, not the file's scene: no lights, cameras or textures from the file
            _sourceNode = new MeshInstance3D { Mesh = DisplayMesh.ForSource(_p.Source), MaterialOverride = DisplayMesh.Material };
            AddChild(_sourceNode);
            _sourceNode.Visible = _showSource.ButtonPressed;
            _orbit.Frame(_p.Source.Mesh.Bounds());
            UpdateVoxelInfo();
            Log($"Imported {System.IO.Path.GetFileName(path)}");
            return Task.CompletedTask;
        });
        UpdateChrome();
        if (_p.Source != null)
        {
            _triInfo.Text = $"Source: {_p.Source.Mesh.TriCount:N0} triangles";
            _triInfo.Modulate = Colors.White;
            _retopoWin.Visible = true;
        }
    }

    void ClearAll()
    {
        _sourceNode?.QueueFree(); _sourceNode = null;
        _retopoMI?.QueueFree(); _retopoMI = null;
        ClearPreview();
        _p.Welded = null; _p.Rig = null; _p.WeldedBones = null; _p.Split = null;
        _gizmo.SetRig(null, 1);
        _boneList.Clear();
        _weightsStale = true;
    }

    async Task OnRetopoPressed()
    {
        _retopoWin.Visible = false;
        _p.S.VoxelRes = (int)_voxel.Value;
        _p.S.Close = (int)_close.Value;
        _p.S.RelaxIters = (int)_relax.Value;
        _p.S.TargetTris = (int)_tris.Value;
        _p.S.Mode = _retopoMode.Selected == 0 ? RetopoMode.Quad : RetopoMode.Voxel;
        _p.S.Sizing = _spacing.Selected switch { 1 => "uniform", 2 => "adaptive", _ => "auto" };
        await RunBusy("retopology", async () =>
        {
            int before = _p.Source.Mesh.TriCount;
            await Task.Run(() => _p.Retopo());
            InvalidateWeights();
            ShowRetopoStatic();
            _triInfo.Text = $"Triangles: {before:N0} → {_p.Welded.TriCount:N0}";
            if (_p.Quad != null && !_p.UsedFallback)
                _triInfo.Text += $"\n{_p.Quad.Stats.QuadFraction:P0} quads, {_p.Quad.Stats.Irregular} irregular vertices, {_p.SizingChosen} spacing (median quad size {_p.Quad.Rho:0.####})";
            Log(_triInfo.Text);
            if (_p.UsedFallback)
            {
                _triInfo.Text += "\nQuad remesh failed, used voxel fallback: " + _p.FallbackReason;
                _triInfo.Modulate = new Color(1f, 0.6f, 0.4f);
                Log("Quad remesh failed, used voxel fallback: " + _p.FallbackReason);
            }
            else _triInfo.Modulate = Colors.White;
            _showSource.ButtonPressed = false;
            if (_sourceNode != null) _sourceNode.Visible = false;
        });
    }

    async Task OnFitPressed()
    {
        _p.S.Rig = _rigPreset.GetItemText(_rigPreset.Selected);
        await FitWith(_p.S.Rig);
    }

    async Task FitWith(string presetOrPath)
    {
        _p.S.Rig = presetOrPath;
        await RunBusy("fit skeleton", async () =>
        {
            await Task.Run(() => _p.FitSkeleton());
            OnRigChanged();
        });
    }

    async Task OnCenterPressed()
    {
        await RunBusy("center joints", async () =>
        {
            float h = _p.Source.Mesh.Bounds().Size.Y;
            await Task.Run(() => HumanoidTemplate.CenterInVolume(_p.Rig, _p.SourceBvh, h, s => Log(s)));
            _gizmo.Refresh();
            InvalidateWeights();
        });
    }

    void OnRigChanged(int select = -1)
    {
        _gizmo.SetRig(_p.Rig, _p.Source.Mesh.Bounds().Size.Y);
        _boneList.Clear();
        foreach (var b in _p.Rig.Bones) _boneList.AddItem(b.Name);
        if (select >= 0 && select < _p.Rig.Bones.Count) { _boneList.Select(select); _gizmo.Select(select); }
        SyncBoneEditor();
        InvalidateWeights();
        UpdateButtons();
    }

    // ------------------------------------------------------------------ skeleton editing

    /// <summary>Runs a structural rig edit; errors go to the log instead of throwing.</summary>
    void Edit(string what, Func<int> edit)
    {
        if (_p.Rig == null) return;
        try
        {
            int select = edit();
            Log($"Rig: {what}");
            OnRigChanged(select);
        }
        catch (InvalidOperationException e) { Log("Rig edit refused: " + e.Message); }
    }

    float MinBoneLength => _p.Source.Mesh.Bounds().Size.Y * 0.04f;

    void OnAddBone() => Edit("added bone", () => RigEdit.AddChild(_p.Rig, _gizmo.Selected, "Bone", MinBoneLength));

    void OnDeleteBone()
    {
        int b = _gizmo.Selected;
        string name = _p.Rig.Bones[b].Name;
        Edit($"deleted {name}", () => { int parent = _p.Rig.Bones[b].Parent; RigEdit.Remove(_p.Rig, b); return Math.Min(parent, _p.Rig.Bones.Count - 1); });
    }

    void OnSymmetrize() => Edit("mirrored Left* bones to Right*", () =>
    {
        int n = RigEdit.Symmetrize(_p.Rig, _p.Rig.Bones[0].Head.X);
        if (n > 0) Log($"Rig: created {n} mirrored bone(s)");
        return _gizmo.Selected;
    });

    void OnRenameBone()
    {
        int b = _gizmo.Selected;
        var name = _renameEdit.Text;
        Edit($"renamed {_p.Rig.Bones[b].Name} to {name}", () => { RigEdit.Rename(_p.Rig, b, name); return b; });
    }

    void OnReparent(int item)
    {
        if (_syncingBoneUi || _p.Rig == null) return;
        int b = _gizmo.Selected;
        int newParent = _p.Rig.Find(_parentList.GetItemText(item));
        if (b <= 0 || newParent < 0 || newParent == _p.Rig.Bones[b].Parent) return;
        string name = _p.Rig.Bones[b].Name;
        Edit($"{name} now under {_p.Rig.Bones[newParent].Name}", () => { RigEdit.Reparent(_p.Rig, b, newParent); return _p.Rig.Find(name); });
    }

    void OnDeformToggled(bool on)
    {
        if (_syncingBoneUi || _p.Rig == null || _gizmo.Selected < 0) return;
        _p.Rig.Bones[_gizmo.Selected].Deform = on;
        Log($"Rig: {_p.Rig.Bones[_gizmo.Selected].Name} {(on ? "deforms" : "no longer deforms")}");
        InvalidateWeights();
    }

    /// <summary>Shows the selected bone's name, parent and deform flag in the editor controls.</summary>
    void SyncBoneEditor()
    {
        _syncingBoneUi = true;
        _parentList.Clear();
        int b = _gizmo.Selected;
        if (_p.Rig != null && b >= 0 && b < _p.Rig.Bones.Count)
        {
            var bone = _p.Rig.Bones[b];
            _renameEdit.Text = bone.Name;
            _deformCheck.ButtonPressed = bone.Deform;
            if (b == 0) _parentList.AddItem("(root)");
            else
                for (int i = 0; i < _p.Rig.Bones.Count; i++)
                {
                    if (RigEdit.IsDescendant(_p.Rig, i, b)) continue;   // can't parent under itself or its children
                    _parentList.AddItem(_p.Rig.Bones[i].Name);
                    if (i == bone.Parent) _parentList.Select(_parentList.ItemCount - 1);
                }
        }
        _syncingBoneUi = false;
        UpdateButtons();
    }

    void OnSavePreset()
    {
        var name = _presetName.Text.Trim();
        if (name.Length == 0 || name.IndexOfAny(System.IO.Path.GetInvalidFileNameChars()) >= 0) { Log("Give the preset a file-safe name first."); return; }
        var path = RigPreset.Save(_p.Rig, _p.Source.Mesh.Bounds(), name, null, _p.RigPresetData?.Description, _p.RigPresetData?.Poses);
        Log($"Saved rig preset to {path}");
        RefreshPresetList(name);
    }

    void RefreshPresetList(string select = null)
    {
        _rigPreset.Clear();
        var names = RigPreset.List();
        foreach (var n in names) _rigPreset.AddItem(n);
        if (select != null && names.IndexOf(select) is int i and >= 0) _rigPreset.Select(i);
    }

    void OnLoadPresetPressed()
    {
        var fd = OpenDialog("Load rig preset", "*.json ; Rig preset", "user://rigs");
        fd.FileSelected += path => { _ = FitWith(ProjectSettings.GlobalizePath(path)); fd.QueueFree(); };
    }

    /// <summary>
    /// Open-file dialog. Desktop: the system dialog over the whole filesystem. Mobile: Godot's own dialog inside the
    /// app's folder (visible in the Files app), starting in the given subfolder.
    /// </summary>
    FileDialog OpenDialog(string title, string filter, string mobileDir)
    {
        var fd = new FileDialog { FileMode = FileDialog.FileModeEnum.OpenFile, Filters = new[] { filter }, Title = title };
        if (_touchUi)
        {
            fd.Access = FileDialog.AccessEnum.Userdata;
            fd.UseNativeDialog = false;
            fd.CurrentDir = mobileDir;
        }
        else
        {
            fd.Access = FileDialog.AccessEnum.Filesystem;
            fd.UseNativeDialog = true;
        }
        AddChild(fd);
        fd.Canceled += () => fd.QueueFree();
        fd.PopupCentered(new Vector2I(900, 600));
        return fd;
    }

    void InvalidateWeights()
    {
        if (!_weightsStale && _p.WeldedBones != null) Log("Skeleton/mesh changed: weights need recomputing.");
        _weightsStale = true;
        if (_previewRoot != null) { ClearPreview(); ShowRetopoStatic(); }
        UpdateButtons();
    }

    async Task OnWeightsPressed()
    {
        _p.S.Falloff = (float)_falloff.Value;
        await RunBusy("weights", async () =>
        {
            await Task.Run(() => _p.ComputeWeights());
            _weightsStale = false;
            BuildPreview();
        });
    }

    async void OnExportPressed()
    {
        if (_touchUi)
        {
            var name = (_sourcePath != null ? System.IO.Path.GetFileNameWithoutExtension(_sourcePath) : "model") + "_rigged.glb";
            var dest = System.IO.Path.Combine(ProjectSettings.GlobalizePath("user://Exports"), name);
            await ExportTo(dest);
            Log($"Saved to Files › On My iPad › ModelMate › Exports › {name}");
            // then offer it to other apps (AirDrop, Files, Nomad, …) through the system share sheet
            if (_mobile && System.IO.File.Exists(dest))
            {
                var r = _btnExport.GetGlobalRect();   // logical pixels = points (UI scale = screen scale)
                if (!IosBridge.Share(dest, r.Position.X, r.Position.Y, r.Size.X, r.Size.Y)) Log("Sharing is not available here.");
            }
            return;
        }
        var fd = new FileDialog
        {
            FileMode = FileDialog.FileModeEnum.SaveFile, Access = FileDialog.AccessEnum.Filesystem,
            Filters = new[] { "*.glb ; Binary glTF" }, UseNativeDialog = true, Title = "Export .glb",
            CurrentFile = (_sourcePath != null ? System.IO.Path.GetFileNameWithoutExtension(_sourcePath) : "model") + "_rigged.glb",
        };
        AddChild(fd);
        var tcs = new TaskCompletionSource<string>();
        fd.FileSelected += path => tcs.TrySetResult(path);
        fd.Canceled += () => tcs.TrySetResult(null);
        fd.PopupCentered(new Vector2I(900, 600));
        var outPath = await tcs.Task;
        fd.QueueFree();
        if (outPath == null) return;
        if (!outPath.EndsWith(".glb", StringComparison.OrdinalIgnoreCase)) outPath += ".glb";
        await ExportTo(outPath);
    }

    bool? _lastExportOk;

    async Task ExportTo(string outPath)
    {
        System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(outPath));
        _lastExportOk = null;
        _p.S.TexSize = _texSizeValue;
        await RunBusy("export", async () =>
        {
            if (_p.Rig != null && (_weightsStale || _p.WeldedBones == null))
            {
                await BusyStep("Computing weights…");
                _p.S.Falloff = (float)_falloff.Value;
                await Task.Run(() => _p.ComputeWeights());
                _weightsStale = false;
                BuildPreview();
            }
            await BusyStep("Unwrapping UVs…");
            if (UvUnwrap.Mode == "xatlas") _p.Unwrap();    // xatlas goes through the Godot API: main thread
            else await Task.Run(() => _p.Unwrap());        // the built-in unwrapper is plain C#
            await BusyStep("Writing and checking the file…");
            var cl = new CheckList();
            Verify.PreExport(cl, _p.Welded, _p.Split, _p.SplitOrigin, _p.SplitBones, _p.SplitWeights, _p.AppliedTriBudget);
            ChartUnwrap.CheckLayout(cl, $"UV layout ({UvUnwrap.LastUsed})", _p.Split, _p.S.TexSize, UvUnwrap.LastUsed != "xatlas");
            Verify.QuadMesh(cl, _p);
            Verify.LimbSides(cl, _p);
            var root = _p.BuildScene(System.IO.Path.GetFileNameWithoutExtension(outPath));
            Exporter.ExportGlb(root, outPath);
            root.Free();
            Verify.RoundTrip(cl, outPath, Verify.IsHumanoid(_p.Rig), _p.Rigged);
            Log($"Exported {outPath}" + (_p.Rigged ? "" : " (static mesh: no skeleton was fitted)"));
            cl.Print(Log);
            Log(cl.AllOk ? "All checks passed." : "Some checks FAILED — see above.");
            _lastExportOk = cl.AllOk;
        });
        if (_lastExportOk == false)
        {
            _status.Text = "Exported, but some checks FAILED (see Console)";
            if (!_console.Visible) _console.PopupCentered();
        }
    }

    // ------------------------------------------------------------------ display

    /// <summary>The remeshed model: plain grey, or the selected bone's weights when the heatmap is on.</summary>
    ArrayMesh RetopoDisplayMesh(bool skinned) =>
        DisplayMesh.Build(_p.Welded, HeatColors(), skinned ? _p.WeldedBones : null, skinned ? _p.WeldedWeights : null);

    void ShowRetopoStatic()
    {
        _retopoMI?.QueueFree();
        _retopoMI = new MeshInstance3D { Mesh = RetopoDisplayMesh(false), MaterialOverride = DisplayMesh.Material };
        AddChild(_retopoMI);
        RefreshVisibility();
    }

    void ClearPreview()
    {
        _previewRoot?.QueueFree();
        _previewRoot = null; _previewSkel = null; _previewMI = null;
        if (_gizmo != null) _gizmo.PoseSkeleton = null;
        _poseEuler.Clear();
    }

    void BuildPreview()
    {
        ClearPreview();
        var rig = _p.Rig;
        _previewRoot = new Node3D();
        _previewSkel = new Skeleton3D();
        _previewRoot.AddChild(_previewSkel);
        var globals = new Transform3D[rig.Bones.Count];
        for (int i = 0; i < rig.Bones.Count; i++) { _previewSkel.AddBone(rig.Bones[i].Name); globals[i] = rig.GlobalRest(i); }
        var skin = new Skin();
        for (int i = 0; i < rig.Bones.Count; i++)
        {
            int par = rig.Bones[i].Parent;
            if (par >= 0) _previewSkel.SetBoneParent(i, par);
            _previewSkel.SetBoneRest(i, par >= 0 ? globals[par].AffineInverse() * globals[i] : globals[i]);
            skin.AddBind(i, globals[i].AffineInverse());
        }
        _previewSkel.ResetBonePoses();
        _previewMI = new MeshInstance3D
        {
            Mesh = RetopoDisplayMesh(true),
            MaterialOverride = DisplayMesh.Material, Skin = skin,
        };
        _previewSkel.AddChild(_previewMI);
        _previewMI.Skeleton = new NodePath("..");
        AddChild(_previewRoot);
        _gizmo.PoseSkeleton = _previewSkel;
        _retopoMI?.QueueFree(); _retopoMI = null;
        RefreshVisibility();
    }

    void RefreshVisibility()
    {
        bool show = _showRetopo.ButtonPressed;
        if (_retopoMI != null) _retopoMI.Visible = show;
        if (_previewRoot != null) _previewRoot.Visible = show;
    }

    Color[] HeatColors()
    {
        var m = _p.Welded;
        var cols = new Color[m.VertexCount];
        int bone = _gizmo.Selected;
        bool heat = _heat != null && _heat.ButtonPressed && bone >= 0 && _p.WeldedBones != null && !_weightsStale;
        if (!heat) return null;
        for (int v = 0; v < cols.Length; v++)
        {
            float w = 0;
            for (int k = 0; k < 4; k++) if (_p.WeldedBones[4 * v + k] == bone) w += _p.WeldedWeights[4 * v + k];
            // blue → green → red
            cols[v] = w < 0.5f ? new Color(0.1f, 0.2f + 1.4f * w, 1f - 1.6f * w) : new Color(2f * (w - 0.5f) + 0.1f, 1f - 1.6f * (w - 0.5f), 0.1f);
            if (w <= 0) cols[v] = new Color(0.12f, 0.15f, 0.45f);
        }
        return cols;
    }

    void UpdateHeatmap()
    {
        if (_p.Welded == null) return;
        if (_previewMI != null)
            _previewMI.Mesh = RetopoDisplayMesh(true);
        else if (_retopoMI != null)
            _retopoMI.Mesh = RetopoDisplayMesh(false);
    }

    bool _syncingSliders;

    void OnBoneSelected(int bone)
    {
        SyncBoneEditor();
        _syncingSliders = true;
        var e = _poseEuler.TryGetValue(bone, out var v) ? v : Vector3.Zero;
        _rx.Value = e.X; _ry.Value = e.Y; _rz.Value = e.Z;
        _syncingSliders = false;
        if (_heat.ButtonPressed) UpdateHeatmap();
    }

    void OnPoseSlider()
    {
        if (_syncingSliders || _previewSkel == null) return;
        int b = _gizmo.Selected;
        if (b < 0) return;
        var e = new Vector3((float)_rx.Value, (float)_ry.Value, (float)_rz.Value);
        _poseEuler[b] = e;
        var rest = _previewSkel.GetBoneRest(b);
        var rot = Basis.FromEuler(new Vector3(Mathf.DegToRad(e.X), Mathf.DegToRad(e.Y), Mathf.DegToRad(e.Z)));
        _previewSkel.SetBonePoseRotation(b, (rest.Basis * rot).GetRotationQuaternion());
    }

    void ResetPose()
    {
        _previewSkel?.ResetBonePoses();
        _poseEuler.Clear();
        OnBoneSelected(_gizmo.Selected);
    }
}
