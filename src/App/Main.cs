using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Godot;
using ModelMate.Core;

namespace ModelMate.App;

/// <summary>
/// ModelMate UI. Steps: Import → Retopologize → Skeleton (drag joints) → Weights → Preview pose → Export .glb.
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
    CheckBox _faceNegZ, _showSource, _showRetopo, _wire, _mirror, _showSkel, _heat;
    OptionButton _retopoMode, _spacing, _rigPreset, _parentList, _boneList, _texSize;
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
        if (si >= 0 && si + 2 < args.Length) { await UiSmoke(args[si + 1], args[si + 2]); return; }
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
        try { _peakMem = Math.Max(_peakMem, System.Environment.WorkingSet); } catch { /* not supported */ }
    }

    string MemoryText()
    {
        try { return $"memory {System.Environment.WorkingSet / 1048576} MB, peak {_peakMem / 1048576} MB"; }
        catch { return "memory n/a"; }
    }

    /// <summary>Drives the UI steps programmatically (same code paths as the buttons), then screenshots.</summary>
    async Task UiSmoke(string input, string outDir)
    {
        outDir = ProjectSettings.GlobalizePath(outDir);
        await Import(ProjectSettings.GlobalizePath(input));
        _tris.Value = 8000;
        _rigPreset.Select(RigPreset.List().IndexOf("humanoid"));
        await OnFitPressed();
        await OnRetopoPressed();
        await OnWeightsPressed();
        _boneList.Select(_p.Rig.Find("LeftUpperArm"));
        _gizmo.Select(_p.Rig.Find("LeftUpperArm"));
        _heat.ButtonPressed = true;
        OnBoneSelected(_gizmo.Selected);
        await Shot(System.IO.Path.Combine(outDir, "ui_heat.png"));
        _heat.ButtonPressed = false;
        Render.ApplyTestPose(_previewSkel);
        await Shot(System.IO.Path.Combine(outDir, "ui_pose.png"));
        ResetPose();
        // skeleton editor through the same handlers as the buttons
        int hand = _p.Rig.Find("LeftHand");
        _boneList.Select(hand); _gizmo.Select(hand); OnBoneSelected(hand);
        OnAddBone();                                                   // child of LeftHand
        _renameEdit.Text = "LeftClaw"; OnRenameBone();
        OnSymmetrize();                                                // creates RightClaw
        bool editsOk = _p.Rig.Find("LeftClaw") >= 0 && _p.Rig.Find("RightClaw") >= 0 &&
                       _p.Rig.Bones[_p.Rig.Find("RightClaw")].Parent == _p.Rig.Find("RightHand");
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
        async Task Touch(int idx, Vector2 pos, bool down) { Input.ParseInputEvent(new InputEventScreenTouch { Index = idx, Position = pos, Pressed = down }); await Frames(); }
        async Task Drag(int idx, Vector2 from, Vector2 to, int steps = 6)
        {
            for (int s = 1; s <= steps; s++)
            {
                var a = from.Lerp(to, (s - 1) / (float)steps); var b = from.Lerp(to, s / (float)steps);
                Input.ParseInputEvent(new InputEventScreenDrag { Index = idx, Position = b, Relative = b - a, ScreenRelative = b - a });
                await Frames(1);
            }
            await Frames();
        }

        await Import(ProjectSettings.GlobalizePath("res://test/humanoid.glb"));
        _rigPreset.Select(RigPreset.List().IndexOf("humanoid"));
        await OnFitPressed();
        await Frames(5);
        GD.Print($"Touch test: viewport {GetViewport().GetVisibleRect().Size}, pick radius {_gizmo.PickRadius}");

        // 1. one finger on empty space (top-left sky) orbits
        float yaw0 = _orbit.Yaw;
        var p0 = new Vector2(120, 120);
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
        var pp = new Vector2(vp.X - 120, 30);   // over the "1 · Import" header, inside the panel (nothing to toggle)
        await Touch(0, pp, true);
        await Drag(0, pp, pp + new Vector2(-150, 0));
        await Touch(0, pp + new Vector2(-150, 0), false);
        Check(MathF.Abs(_orbit.Yaw - yaw3) < 1e-4f, $"drag over the panel does not orbit (yaw {yaw3:F3} -> {_orbit.Yaw:F3})");
        Check(_p.Rig.Bones[hand].Deform, "LeftHand still deforms (no control toggled by the test)");

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
            AmbientLightSource = Godot.Environment.AmbientSource.Color, AmbientLightColor = new Color(0.45f, 0.45f, 0.5f),
        };
        AddChild(new WorldEnvironment { Environment = env });
        AddChild(new DirectionalLight3D { RotationDegrees = new Vector3(-45, 35, 0), ShadowEnabled = false });
        AddChild(new DirectionalLight3D { RotationDegrees = new Vector3(-20, 210, 0), LightEnergy = 0.4f });
        _orbit = new OrbitCamera();
        AddChild(_orbit);
        _gizmo = new BoneGizmo();
        if (_touchUi) _gizmo.PickRadius = 28;   // fingertip-sized (logical pixels, after UI scale)
        AddChild(_gizmo);
        _gizmo.Cam = null;
        _gizmo.SelectionChanged += b => { if (b >= 0 && b < _boneList.ItemCount) _boneList.Select(b); OnBoneSelected(b); };
        _gizmo.Edited += () => { InvalidateWeights(); };
        _gizmo.EditBlocked += () => Log("Skeleton is posed: press 'Reset pose' to edit joints.");
        // floor
        _floor = new MeshInstance3D
        {
            Mesh = new PlaneMesh { Size = new Vector2(4, 4) },
            MaterialOverride = new StandardMaterial3D { AlbedoColor = new Color(0.22f, 0.23f, 0.26f) },
        };
        AddChild(_floor);
    }

    MeshInstance3D _floor;

    // ------------------------------------------------------------------ UI

    void BuildUi()
    {
        var layer = new CanvasLayer();
        AddChild(layer);
        var panel = new PanelContainer
        {
            AnchorLeft = 1, AnchorRight = 1, AnchorBottom = 1, OffsetLeft = -420,
        };
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

        _status = new Label { Text = "Idle" };

        Header(vb, "1 · Import");
        _faceNegZ = Check(vb, "Model faces -Z (rotate 180° on import)", false, _ => { });
        var ib = new HBoxContainer();
        vb.AddChild(ib);
        var bImport = Btn(ib, "Import glTF…", OnImportPressed);
        var bFrame = Btn(ib, "Frame model", () => { if (_p.Source != null) _orbit.Frame(_p.Source.Mesh.Bounds()); });
        bFrame.TooltipText = "Re-centre the camera on the model";
        _showSource = Check(vb, "Show source", true, v => { if (_sourceNode != null) _sourceNode.Visible = v; });

        Header(vb, "2 · Skeleton");
        _rigPreset = Option(vb, "Rig preset", RigPreset.List().ToArray());
        _btnFit = Btn(vb, "Fit skeleton", () => _ = OnFitPressed());
        _btnLoadPreset = Btn(vb, "Load preset file…", OnLoadPresetPressed);
        _btnCenter = Btn(vb, "Center joints in volume", () => _ = OnCenterPressed());
        _mirror = Check(vb, "Mirror X edits", true, v => _gizmo.Mirror = v);
        _showSkel = Check(vb, "Show skeleton", true, v => _gizmo.Visible = v);
        _boneList = Option(vb, "Bone", Array.Empty<string>());
        _boneList.ItemSelected += i => { _gizmo.Select((int)i); OnBoneSelected((int)i); };
        var eb = new HBoxContainer();
        vb.AddChild(eb);
        _btnAddBone = Btn(eb, "Add child", OnAddBone);
        _btnDelBone = Btn(eb, "Delete", OnDeleteBone);
        _btnSym = Btn(eb, "Symmetrize", OnSymmetrize);
        _btnSym.TooltipText = "Copy every Left* bone onto its Right* partner (mirrored), creating missing partners";
        _btnAddBone.TooltipText = "Add a bone under the selected one";
        if (_touchUi) Hint(vb, "Symmetrize copies every Left* bone onto its Right* partner, creating missing ones.");
        _btnDelBone.TooltipText = "Delete the selected bone; its children move to its parent";
        var rn = new HBoxContainer();
        vb.AddChild(rn);
        _renameEdit = new LineEdit { PlaceholderText = "bone name", SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        rn.AddChild(_renameEdit);
        _btnRename = Btn(rn, "Rename", OnRenameBone);
        _btnRename.CustomMinimumSize = new Vector2(90, 0);
        _btnRename.SizeFlagsHorizontal = Control.SizeFlags.ShrinkEnd;
        _parentList = Option(vb, "Parent", Array.Empty<string>());
        _parentList.ItemSelected += i => OnReparent((int)i);
        _deformCheck = Check(vb, "Deforms mesh (gets weights)", true, OnDeformToggled);
        var sp = new HBoxContainer();
        vb.AddChild(sp);
        _presetName = new LineEdit { PlaceholderText = "preset name", SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        sp.AddChild(_presetName);
        _btnSavePreset = Btn(sp, "Save", OnSavePreset);
        _btnSavePreset.TooltipText = "Save this skeleton as a reusable preset (joint positions relative to the model's bounds)";
        _btnSavePreset.CustomMinimumSize = new Vector2(90, 0);
        _btnSavePreset.SizeFlagsHorizontal = Control.SizeFlags.ShrinkEnd;

        Header(vb, "3 · Retopology");
        _retopoMode = Option(vb, "Mode", new[] { "Quad (recommended)", "Voxel + decimate" });
        _retopoMode.ItemSelected += _ => UpdateVoxelInfo();
        _spacing = Option(vb, "Spacing (quad mode)", new[] { "Auto (recommended)", "Uniform", "Adaptive" });
        _spacing.TooltipText = "Adaptive spends more quads where the surface curves. Auto builds both and keeps the closer match.";
        if (_touchUi) Hint(vb, "Auto builds uniform and adaptive spacing and keeps the closer match.");
        _tris = Spin(vb, "Target triangles", 200, 200000, 8000, 100, _ => UpdateVoxelInfo());
        _voxel = Spin(vb, "Voxel resolution (voxel mode)", 64, 512, 256, 16, _ => UpdateVoxelInfo());
        _voxelInfo = new Label { Modulate = new Color(1, 1, 1, 0.7f), AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(300, 0) };
        vb.AddChild(_voxelInfo);
        _close = Spin(vb, "Gap closing (voxels)", 0, 3, 1, 1);
        _relax = Spin(vb, "Relax iterations", 0, 10, 4, 1);
        _btnRetopo = Btn(vb, "Retopologize", () => _ = OnRetopoPressed());
        _triInfo = new Label();
        vb.AddChild(_triInfo);
        _showRetopo = Check(vb, "Show retopo mesh", true, _ => RefreshVisibility());
        _wire = Check(vb, "Wireframe", false, v => GetViewport().DebugDraw = v ? Viewport.DebugDrawEnum.Wireframe : Viewport.DebugDrawEnum.Disabled);

        Header(vb, "4 · Weights");
        _falloff = Spin(vb, "Falloff exponent", 1, 8, 3, 0.5);
        _btnWeights = Btn(vb, "Compute weights", () => _ = OnWeightsPressed());
        _heat = Check(vb, "Show selected bone's weights", false, _ => UpdateHeatmap());

        Header(vb, "5 · Preview pose");
        _rx = Slider(vb, "Rotate X", OnPoseSlider);
        _ry = Slider(vb, "Rotate Y", OnPoseSlider);
        _rz = Slider(vb, "Rotate Z", OnPoseSlider);
        var hb = new HBoxContainer();
        vb.AddChild(hb);
        _btnTestPose = Btn(hb, "Test pose", () => { if (_previewSkel != null) Render.ApplyTestPose(_previewSkel); });
        _btnResetPose = Btn(hb, "Reset pose", ResetPose);

        Header(vb, "6 · Export");
        _texSize = Option(vb, "Texture size", new[] { "512", "1024", "2048", "4096" });
        _texSize.Select(1);
        _btnExport = Btn(vb, "Export .glb…", OnExportPressed);

        vb.AddChild(new HSeparator());
        vb.AddChild(_status);
        _log = new RichTextLabel
        {
            ScrollFollowing = true, CustomMinimumSize = new Vector2(0, 260), SelectionEnabled = true,
            SizeFlagsVertical = Control.SizeFlags.ExpandFill,
        };
        _log.AddThemeFontSizeOverride("normal_font_size", 11);
        vb.AddChild(_log);
        UpdateVoxelInfo();
        if (_touchUi)
        {
            EnlargeForTouch(vb);
            // keep clear of rounded corners / home indicator
            var safe = DisplayServer.GetDisplaySafeArea();
            var screen = DisplayServer.ScreenGetSize();
            float sc = GetWindow().ContentScaleFactor;
            if (_mobile && safe.Size.X > 0)
            {
                panel.OffsetTop = safe.Position.Y / sc;
                panel.OffsetBottom = -(screen.Y - safe.End.Y) / sc;
                panel.OffsetRight = -(screen.X - safe.End.X) / sc;
                panel.OffsetLeft = panel.OffsetRight - 420;
            }
        }
    }

    static void Hint(Container c, string text)
    {
        var l = new Label { Text = text, Modulate = new Color(1, 1, 1, 0.7f), AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(300, 0) };
        l.AddThemeFontSizeOverride("font_size", 13);
        c.AddChild(l);
    }

    static void Header(Container c, string text)
    {
        c.AddChild(new HSeparator());
        var l = new Label { Text = text };
        l.AddThemeFontSizeOverride("font_size", 15);
        c.AddChild(l);
    }

    Button Btn(Container c, string text, Action onPress)
    {
        var b = new Button { Text = text, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
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

    public override void _Process(double delta)
    {
        if (_log == null) return;
        while (_logQ.TryDequeue(out var s))
        {
            _log.AddText(s + "\n");
            GD.Print(s);
        }
        if (_gizmo.Cam == null && _orbit.Cam != null) _gizmo.Cam = _orbit.Cam;
        _memSampleT += delta;
        if (_memSampleT > 0.25) { _memSampleT = 0; SampleMemory(); }
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
        _btnWeights.Disabled = !rig || !retopo;
        _btnTestPose.Disabled = !w;
        _btnResetPose.Disabled = !w;
        _btnExport.Disabled = !rig || !retopo;
    }

    async Task RunBusy(string what, Func<Task> work)
    {
        if (_busy) return;
        _busy = true;
        _status.Text = "Working: " + what + "…" + (_mobile ? " (keep ModelMate open: switching apps pauses it)" : "");
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
            _status.Text = "Failed: " + what;
        }
        finally
        {
            _busy = false;
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
            _p.Import(path, true, out var scene);
            _sourcePath = path;
            _sourceNode = (Node3D)scene;
            AddChild(_sourceNode);
            _sourceNode.Visible = _showSource.ButtonPressed;
            var box = _p.Source.Mesh.Bounds();
            _orbit.Frame(box);
            float ext = box.Size.Length();
            _floor.Position = new Vector3(box.GetCenter().X, box.Position.Y - ext * 0.002f, box.GetCenter().Z);
            ((PlaneMesh)_floor.Mesh).Size = new Vector2(ext * 2, ext * 2);
            UpdateVoxelInfo();
            Log($"Imported {System.IO.Path.GetFileName(path)}");
            return Task.CompletedTask;
        });
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
            if (_p.UsedFallback)
            {
                _triInfo.Text += "\nQuad remesh failed, used voxel fallback: " + _p.FallbackReason;
                _triInfo.Modulate = new Color(1f, 0.6f, 0.4f);
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
        _lastExportOk = null;
        _p.S.TexSize = int.Parse(_texSize.GetItemText(_texSize.Selected));
        await RunBusy("export", async () =>
        {
            if (_weightsStale || _p.WeldedBones == null)
            {
                _p.S.Falloff = (float)_falloff.Value;
                await Task.Run(() => _p.ComputeWeights());
                _weightsStale = false;
                BuildPreview();
            }
            _status.Text = "Working: UV unwrap…";
            await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
            _p.Unwrap();                                   // main thread (Godot API)
            _status.Text = "Working: texture bake…";
            await Task.Run(() => _p.Bake());
            var cl = new CheckList();
            Verify.PreExport(cl, _p.Welded, _p.Split, _p.SplitOrigin, _p.SplitBones, _p.SplitWeights, _p.AppliedTriBudget);
            ChartUnwrap.CheckLayout(cl, $"UV layout ({UvUnwrap.LastUsed})", _p.Split, _p.S.TexSize, UvUnwrap.LastUsed == "charts");
            Verify.BakedTexture(cl, _p.AlbedoRgba);
            Verify.QuadMesh(cl, _p);
            Verify.LimbSides(cl, _p);
            var root = _p.BuildScene(System.IO.Path.GetFileNameWithoutExtension(outPath));
            Exporter.ExportGlb(root, outPath);
            root.Free();
            Verify.RoundTrip(cl, outPath, Verify.IsHumanoid(_p.Rig), _p.S.NormalMap);
            Log($"Exported {outPath}");
            cl.Print(Log);
            Log(cl.AllOk ? "All checks passed." : "Some checks FAILED — see above.");
            _lastExportOk = cl.AllOk;
        });
        if (_lastExportOk == false) _status.Text = "Exported, but some checks FAILED (see log)";
    }

    // ------------------------------------------------------------------ display

    static ArrayMesh MakeMesh(MeshData m, Color[] colors, int[] bones, float[] weights)
    {
        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = m.Pos;
        arrays[(int)Mesh.ArrayType.Normal] = m.Nrm;
        if (colors != null) arrays[(int)Mesh.ArrayType.Color] = colors;
        if (bones != null) { arrays[(int)Mesh.ArrayType.Bones] = bones; arrays[(int)Mesh.ArrayType.Weights] = weights; }
        arrays[(int)Mesh.ArrayType.Index] = UvUnwrap.ToGodotWinding(m.Tris);
        var am = new ArrayMesh();
        am.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);
        return am;
    }

    static StandardMaterial3D RetopoMaterial() => new()
    {
        AlbedoColor = new Color(0.75f, 0.76f, 0.8f), Roughness = 0.8f, VertexColorUseAsAlbedo = true,
    };

    void ShowRetopoStatic()
    {
        _retopoMI?.QueueFree();
        _retopoMI = new MeshInstance3D { Mesh = MakeMesh(_p.Welded, HeatColors(), null, null), MaterialOverride = RetopoMaterial() };
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
            Mesh = MakeMesh(_p.Welded, HeatColors(), _p.WeldedBones, _p.WeldedWeights),
            MaterialOverride = RetopoMaterial(), Skin = skin,
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
        for (int v = 0; v < cols.Length; v++)
        {
            if (!heat) { cols[v] = Colors.White; continue; }
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
            _previewMI.Mesh = MakeMesh(_p.Welded, HeatColors(), _p.WeldedBones, _p.WeldedWeights);
        else if (_retopoMI != null)
            _retopoMI.Mesh = MakeMesh(_p.Welded, HeatColors(), null, null);
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
