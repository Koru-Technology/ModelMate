using System;
using Godot;

namespace ModelMate.App;

/// <summary>Orbit camera: right/middle-drag orbit, shift+drag pan, wheel zoom.</summary>
public partial class OrbitCamera : Node3D
{
    public Camera3D Cam;
    public Vector3 Target;
    float _yaw = 0, _pitch = -0.15f, _dist = 4;
    bool _dragging;

    public override void _Ready()
    {
        Cam = new Camera3D { Fov = 45, Near = 0.01f, Far = 500 };
        AddChild(Cam);
        Cam.MakeCurrent();
        Apply();
    }

    public void Frame(Aabb box)
    {
        Target = box.GetCenter();
        _dist = MathF.Max(0.1f, box.Size.Length() * 1.3f);
        Cam.Near = _dist * 0.005f;
        Cam.Far = _dist * 50f;
        Apply();
    }

    void Apply()
    {
        var b = Basis.FromEuler(new Vector3(_pitch, _yaw, 0));
        Cam.GlobalTransform = new Transform3D(b, Target + b.Z * _dist);
    }

    public override void _UnhandledInput(InputEvent e)
    {
        if (e is InputEventMouseButton mb)
        {
            if (mb.ButtonIndex is MouseButton.Right or MouseButton.Middle) _dragging = mb.Pressed;
            if (mb.Pressed && mb.ButtonIndex == MouseButton.WheelUp) { _dist *= 0.9f; Apply(); }
            if (mb.Pressed && mb.ButtonIndex == MouseButton.WheelDown) { _dist /= 0.9f; Apply(); }
        }
        else if (e is InputEventMouseMotion mm && _dragging)
        {
            if (mm.ShiftPressed)
            {
                float s = _dist * 0.0015f;
                var b = Cam.GlobalBasis;
                Target += (-b.X * mm.Relative.X + b.Y * mm.Relative.Y) * s;
            }
            else
            {
                _yaw -= mm.Relative.X * 0.008f;
                _pitch = Math.Clamp(_pitch - mm.Relative.Y * 0.008f, -1.5f, 1.5f);
            }
            Apply();
        }
        else if (e is InputEventPanGesture pg)
        {
            _dist *= 1 + pg.Delta.Y * 0.05f; Apply();
        }
    }
}
