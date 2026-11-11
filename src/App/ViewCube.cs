using System;
using Godot;

namespace ModelMate.App;

/// <summary>
/// Orientation cube: turns with the main camera. Dragging it turns the view; clicking (or tapping) a face turns the
/// camera to look from that side (Front = +Z, the rig templates' forward; Left = +X, the character's left). Rendered
/// in its own small transparent world.
/// </summary>
public partial class ViewCube : SubViewportContainer
{
    public OrbitCamera Orbit;
    SubViewport _vp;
    Camera3D _cam;

    static readonly (Vector3 n, Vector3 up, string label, Color col)[] Faces =
    {
        (Vector3.Back, Vector3.Up, "Front", new Color(0.35f, 0.62f, 0.38f)),        // +Z
        (Vector3.Forward, Vector3.Up, "Back", new Color(0.25f, 0.45f, 0.28f)),      // −Z
        // named from the character, like the rig's bones: its left side (Left* bones) is +X
        (Vector3.Right, Vector3.Up, "Left", new Color(0.72f, 0.33f, 0.33f)),        // +X
        (Vector3.Left, Vector3.Up, "Right", new Color(0.55f, 0.25f, 0.27f)),        // −X
        (Vector3.Up, Vector3.Forward, "Top", new Color(0.36f, 0.52f, 0.78f)),       // +Y
        (Vector3.Down, Vector3.Back, "Bottom", new Color(0.26f, 0.37f, 0.58f)),     // −Y
    };

    public override void _Ready()
    {
        Stretch = true;
        CustomMinimumSize = new Vector2(112, 112);
        MouseFilter = MouseFilterEnum.Stop;
        TooltipText = "Drag to turn the view; click a face to view from that side";
        _vp = new SubViewport { OwnWorld3D = true, TransparentBg = true, Msaa3D = Viewport.Msaa.Msaa4X, Size = new Vector2I(112, 112) };
        AddChild(_vp);
        _cam = new Camera3D { Projection = Camera3D.ProjectionType.Orthogonal, Size = 1.8f, Near = 0.1f, Far = 20 };
        _vp.AddChild(_cam);
        // dark core shows between the faces as edges
        _vp.AddChild(new MeshInstance3D
        {
            Mesh = new BoxMesh { Size = Vector3.One * 0.985f },
            MaterialOverride = new StandardMaterial3D { AlbedoColor = new Color(0.08f, 0.09f, 0.1f), ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded },
        });
        foreach (var (n, up, label, col) in Faces)
        {
            var basis = new Basis(up.Cross(n), up, n);
            _vp.AddChild(new MeshInstance3D
            {
                Mesh = new QuadMesh { Size = Vector2.One * 0.94f },
                MaterialOverride = new StandardMaterial3D { AlbedoColor = col, ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded },
                Transform = new Transform3D(basis, n * 0.5f),
            });
            _vp.AddChild(new Label3D
            {
                Text = label, FontSize = 44, PixelSize = 0.0042f, Modulate = new Color(0.06f, 0.07f, 0.08f),
                OutlineSize = 0, DoubleSided = false, Transform = new Transform3D(basis, n * 0.505f),
            });
        }
    }

    public override void _Process(double delta)
    {
        if (Orbit?.Cam == null) return;
        var b = Orbit.Cam.GlobalBasis;
        _cam.GlobalTransform = new Transform3D(b, b.Z * 5);
    }

    // A press that moves more than a few pixels is a drag: it turns the view as if grabbing the cube. A press that
    // doesn't is a tap: it turns the view to the face under it.
    const float DragThreshold = 6;
    const float DragSpeed = 0.014f;   // radians per pixel: a drag across the cube turns it about 90°
    bool _pressed, _dragging;
    Vector2 _pressPos;

    public override void _GuiInput(InputEvent e)
    {
        if (Orbit == null) return;
        if (e is InputEventMouseButton { ButtonIndex: MouseButton.Left } mb)
        {
            if (mb.Pressed) { _pressed = true; _dragging = false; _pressPos = mb.Position; }
            else if (_pressed)
            {
                _pressed = false;
                if (!_dragging) SnapToFace(mb.Position);
            }
            AcceptEvent();
        }
        else if (e is InputEventMouseMotion mm && _pressed)
        {
            if (!_dragging && mm.Position.DistanceTo(_pressPos) > DragThreshold) _dragging = true;
            if (_dragging) Orbit.Rotate(mm.Relative, DragSpeed);
            AcceptEvent();
        }
    }

    void SnapToFace(Vector2 position)
    {
        var pos = position * ((Vector2)_vp.Size / Size);
        var o = _cam.ProjectRayOrigin(pos);
        var d = _cam.ProjectRayNormal(pos);
        // ray against the unit cube [−½, ½]³ (slabs)
        float t0 = float.MinValue, t1 = float.MaxValue;
        for (int a = 0; a < 3; a++)
        {
            if (MathF.Abs(d[a]) < 1e-9f) { if (MathF.Abs(o[a]) > 0.5f) return; continue; }
            float ta = (-0.5f - o[a]) / d[a], tb = (0.5f - o[a]) / d[a];
            t0 = MathF.Max(t0, MathF.Min(ta, tb));
            t1 = MathF.Min(t1, MathF.Max(ta, tb));
        }
        if (t0 > t1 || t1 < 0) return;
        var p = o + d * t0;
        int ax = 0;
        for (int a = 1; a < 3; a++) if (MathF.Abs(p[a]) > MathF.Abs(p[ax])) ax = a;
        var n = Vector3.Zero;
        n[ax] = MathF.Sign(p[ax]);
        Orbit.ViewFrom(n);
    }
}
