using System;
using Godot;

namespace ModelMate.App;

/// <summary>Orbit camera for mouse, trackpad and touch (see the input section).</summary>
public partial class OrbitCamera : Node3D
{
    public Camera3D Cam;
    public Vector3 Target;
    float _yaw = 0, _pitch = -0.15f, _dist = 4;
    bool _dragging;
    public float Yaw => _yaw;
    public float Distance => _dist;

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

    // ---------------------------------------------------------------- input
    //
    // Mouse: right/middle drag orbits, Shift+drag pans, wheel zooms; left drag on empty space also orbits.
    // Touch: Godot turns the first finger into a left click, and the bone gizmo marks a click handled when it lands
    // on a joint, so a one-finger drag that reaches us started on empty space and orbits. Two fingers pinch-zoom and
    // pan; while they are down (until every finger lifts) orbiting and joint dragging are suspended.

    /// <summary>Fingers currently down that started on the 3D view.</summary>
    public static int TouchCount => _touches.Count;
    /// <summary>True from a second finger landing until all fingers lift.</summary>
    public static bool MultiTouch { get; private set; }
    static readonly System.Collections.Generic.Dictionary<int, Vector2> _touches = new();
    bool _leftDragging;
    float _pinchDist;
    Vector2 _pinchCenter;

    public override void _Input(InputEvent e)
    {
        // releases are tracked even over the UI, so a finger lifted over the panel doesn't stay "down"
        if (e is InputEventScreenTouch st && !st.Pressed)
        {
            _touches.Remove(st.Index);
            if (_touches.Count == 0) MultiTouch = false;
            else if (_touches.Count >= 2) StartPinch();
        }
    }

    public override void _UnhandledInput(InputEvent e)
    {
        if (e is InputEventScreenTouch st && st.Pressed)
        {
            _touches[st.Index] = st.Position;
            if (_touches.Count >= 2) { MultiTouch = true; _leftDragging = false; StartPinch(); }
        }
        else if (e is InputEventScreenDrag sd && _touches.ContainsKey(sd.Index))
        {
            _touches[sd.Index] = sd.Position;
            if (_touches.Count >= 2) Pinch();
        }
        else if (e is InputEventMouseButton mb)
        {
            if (mb.ButtonIndex is MouseButton.Right or MouseButton.Middle) _dragging = mb.Pressed;
            if (mb.ButtonIndex == MouseButton.Left) _leftDragging = mb.Pressed && !MultiTouch;
            if (mb.Pressed && mb.ButtonIndex == MouseButton.WheelUp) { _dist *= 0.9f; Apply(); }
            if (mb.Pressed && mb.ButtonIndex == MouseButton.WheelDown) { _dist /= 0.9f; Apply(); }
        }
        else if (e is InputEventMouseMotion mm && (_dragging || _leftDragging) && !MultiTouch)
        {
            if (mm.ShiftPressed) Pan(mm.Relative);
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
        else if (e is InputEventMagnifyGesture mg && mg.Factor > 0)
        {
            _dist /= mg.Factor; Apply();
        }
    }

    (Vector2 a, Vector2 b) FirstTwo()
    {
        Vector2 a = default, b = default; int k = 0;
        foreach (var p in _touches.Values) { if (k == 0) a = p; else if (k == 1) b = p; if (++k == 2) break; }
        return (a, b);
    }

    void StartPinch()
    {
        var (a, b) = FirstTwo();
        _pinchDist = a.DistanceTo(b);
        _pinchCenter = (a + b) * 0.5f;
    }

    void Pinch()
    {
        var (a, b) = FirstTwo();
        float d = a.DistanceTo(b);
        var c = (a + b) * 0.5f;
        if (_pinchDist > 1 && d > 1) _dist = Math.Clamp(_dist * _pinchDist / d, 1e-3f, 1e5f);
        Pan(c - _pinchCenter);
        _pinchDist = d;
        _pinchCenter = c;
        Apply();
    }

    /// <summary>Moves the target so the point under the cursor/fingers follows them (screen pixels).</summary>
    void Pan(Vector2 delta)
    {
        float h = MathF.Max(1, GetViewport().GetVisibleRect().Size.Y);
        float s = 2 * _dist * MathF.Tan(Mathf.DegToRad(Cam.Fov) * 0.5f) / h;
        var b = Cam.GlobalBasis;
        Target += (-b.X * delta.X + b.Y * delta.Y) * s;
    }
}
