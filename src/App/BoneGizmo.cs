using System;
using System.Collections.Generic;
using Godot;
using ModelMate.Core;

namespace ModelMate.App;

/// <summary>Draws the editable skeleton and lets the user drag joints (left mouse). Mirrors Left/Right edits.</summary>
public partial class BoneGizmo : Node3D
{
    public RigSkeleton Rig;
    public Camera3D Cam;
    public bool Mirror = true;
    public int Selected = -1;
    public event Action<int> SelectionChanged;
    public event Action Edited;
    /// <summary>When set, joints are drawn at the posed skeleton's positions and editing is disabled while posed.</summary>
    public Skeleton3D PoseSkeleton;
    public event Action EditBlocked;
    Transform3D[] _delta;

    readonly List<(int bone, bool tail)> _handles = new();
    readonly List<MeshInstance3D> _spheres = new();
    MeshInstance3D _lines;
    ImmediateMesh _im;
    StandardMaterial3D _matJoint, _matSel, _matLine;
    int _drag = -1;
    float _radius = 0.015f;

    public override void _Ready()
    {
        _matJoint = Mat(new Color(1f, 0.85f, 0.2f));
        _matSel = Mat(new Color(0.2f, 1f, 0.4f));
        _matLine = Mat(new Color(1f, 0.6f, 0.1f));
        _im = new ImmediateMesh();
        _lines = new MeshInstance3D { Mesh = _im, CastShadow = GeometryInstance3D.ShadowCastingSetting.Off };
        AddChild(_lines);
    }

    static StandardMaterial3D Mat(Color c) => new()
    {
        AlbedoColor = c, ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded,
        NoDepthTest = true, RenderPriority = 10, Transparency = BaseMaterial3D.TransparencyEnum.Alpha,
    };

    public void SetRig(RigSkeleton rig, float modelHeight)
    {
        Rig = rig;
        _radius = modelHeight * 0.009f;
        Rebuild();
    }

    void Rebuild()
    {
        foreach (var s in _spheres) s.QueueFree();
        _spheres.Clear();
        _handles.Clear();
        if (Rig == null) { _im.ClearSurfaces(); return; }
        for (int i = 0; i < Rig.Bones.Count; i++)
        {
            _handles.Add((i, false));
            if (Rig.Bones[i].PrimaryChild < 0) _handles.Add((i, true));
        }
        var sm = new SphereMesh { Radius = _radius, Height = _radius * 2, RadialSegments = 12, Rings = 6 };
        foreach (var _ in _handles)
        {
            var mi = new MeshInstance3D { Mesh = sm, CastShadow = GeometryInstance3D.ShadowCastingSetting.Off };
            AddChild(mi);
            _spheres.Add(mi);
        }
        Refresh();
    }

    Vector3 RestHandlePos(int h)
    {
        var (b, tail) = _handles[h];
        return tail ? Rig.Bones[b].TailEnd : Rig.Bones[b].Head;
    }

    Vector3 HandlePos(int h) => _delta != null ? _delta[_handles[h].bone] * RestHandlePos(h) : RestHandlePos(h);

    /// <summary>Per-bone posed*rest⁻¹ transform, or null if the skeleton is at rest.</summary>
    void UpdateDelta()
    {
        _delta = null;
        if (PoseSkeleton == null || !IsInstanceValid(PoseSkeleton) || PoseSkeleton.GetBoneCount() != Rig.Bones.Count) return;
        var d = new Transform3D[Rig.Bones.Count];
        bool posed = false;
        for (int i = 0; i < d.Length; i++)
        {
            d[i] = PoseSkeleton.GetBoneGlobalPose(i) * Rig.GlobalRest(i).AffineInverse();
            if (!d[i].IsEqualApprox(Transform3D.Identity)) posed = true;
        }
        if (posed) _delta = d;
    }

    public bool IsPosed => _delta != null;

    public override void _Process(double delta)
    {
        if (Rig != null && PoseSkeleton != null && Visible) Refresh();
    }

    void SetHandlePos(int h, Vector3 p)
    {
        var (b, tail) = _handles[h];
        if (tail) Rig.Bones[b].TailEnd = p; else Rig.Bones[b].Head = p;
    }

    public void Refresh()
    {
        if (Rig == null) return;
        UpdateDelta();
        for (int h = 0; h < _handles.Count; h++)
        {
            _spheres[h].Position = HandlePos(h);
            _spheres[h].MaterialOverride = _handles[h].bone == Selected ? _matSel : _matJoint;
        }
        _im.ClearSurfaces();
        _im.SurfaceBegin(Mesh.PrimitiveType.Lines, _matLine);
        for (int i = 0; i < Rig.Bones.Count; i++)
        {
            var dh = _delta != null ? _delta[i] : Transform3D.Identity;
            int c = Rig.Bones[i].PrimaryChild;
            var dt = _delta != null && c >= 0 ? _delta[c] : dh;
            _im.SurfaceAddVertex(dh * Rig.Bones[i].Head);
            _im.SurfaceAddVertex(dt * Rig.Tail(i));
        }
        _im.SurfaceEnd();
    }

    public void Select(int bone)
    {
        Selected = bone;
        Refresh();
    }

    /// <summary>Joint pick radius in screen pixels (Main enlarges it for touch screens and UI scale).</summary>
    public float PickRadius = 14;

    public override void _UnhandledInput(InputEvent e)
    {
        if (Rig == null || Cam == null || !Visible) return;
        if (e is InputEventMouseButton mb && mb.ButtonIndex == MouseButton.Left)
        {
            if (mb.Pressed)
            {
                if (OrbitCamera.MultiTouch) return;
                int best = -1; float bd = PickRadius;
                for (int h = 0; h < _handles.Count; h++)
                {
                    var p = HandlePos(h);
                    if (Cam.IsPositionBehind(p)) continue;
                    float d = Cam.UnprojectPosition(p).DistanceTo(mb.Position);
                    if (d < bd) { bd = d; best = h; }
                }
                if (best >= 0)
                {
                    if (IsPosed)
                    {
                        Selected = _handles[best].bone;
                        SelectionChanged?.Invoke(Selected);
                        Refresh();
                        EditBlocked?.Invoke();
                        GetViewport().SetInputAsHandled();
                        return;
                    }
                    _drag = best;
                    Selected = _handles[best].bone;
                    SelectionChanged?.Invoke(Selected);
                    Refresh();
                    GetViewport().SetInputAsHandled();
                }
            }
            else if (_drag >= 0)
            {
                _drag = -1;
                Edited?.Invoke();
            }
        }
        else if (e is InputEventMouseMotion mm && _drag >= 0)
        {
            if (OrbitCamera.MultiTouch)
            {
                // a second finger landed: end the joint drag where it is and let the camera pinch/pan
                _drag = -1;
                Edited?.Invoke();
                return;
            }
            var p = HandlePos(_drag);
            var plane = new Plane(-Cam.GlobalBasis.Z, p);
            var hit = plane.IntersectsRay(Cam.ProjectRayOrigin(mm.Position), Cam.ProjectRayNormal(mm.Position));
            if (hit is Vector3 np)
            {
                SetHandlePos(_drag, np);
                if (Mirror) MirrorHandle(_drag);
                Refresh();
            }
            GetViewport().SetInputAsHandled();
        }
    }

    void MirrorHandle(int h)
    {
        var (b, tail) = _handles[h];
        string name = Rig.Bones[b].Name;
        string other = name.StartsWith("Left") ? "Right" + name[4..] : name.StartsWith("Right") ? "Left" + name[5..] : null;
        float cx = Rig.Bones[0].Head.X;
        var p = HandlePos(h);
        if (other == null)
        {
            // centerline bone: keep it on the symmetry plane
            p.X = cx;
            SetHandlePos(h, p);
            return;
        }
        int ob = Rig.Find(other);
        if (ob < 0) return;
        var m = new Vector3(2 * cx - p.X, p.Y, p.Z);
        if (tail) Rig.Bones[ob].TailEnd = m; else Rig.Bones[ob].Head = m;
    }
}
