using System;
using System.Collections.Concurrent;
using System.IO;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace ModelMate.App;

/// <summary>
/// iOS integration through the Objective-C runtime (no native plugin to build):
/// <list type="bullet">
/// <item>Receiving: a tiny Objective-C class, created at startup, is registered as a service with Godot's
/// application delegate (<c>+[GDTApplicationDelegate addService:]</c>), which forwards app and scene callbacks to its
/// services. When another app shares a .glb/.gltf to ModelMate ("Open in"/"Copy to ModelMate"), iOS calls
/// <c>scene:openURLContexts:</c> (app running) or <c>scene:willConnectToSession:options:</c> (app launched by the
/// share); the file is copied into Documents/Models and queued for import. Files iOS drops in Documents/Inbox are
/// swept up too (<see cref="SweepInbox"/>).</item>
/// <item>Sending: <see cref="Share"/> presents the system share sheet (UIActivityViewController) for a file.</item>
/// </list>
/// The document types that make iOS offer ModelMate for glTF files are declared in the export preset's Info.plist
/// additions. Everything here is a no-op off iOS.
/// </summary>
public static unsafe class IosBridge
{
    /// <summary>Files received from other apps (absolute paths in Documents/Models), oldest first.</summary>
    public static readonly ConcurrentQueue<string> Received = new();
    /// <summary>Diagnostics from the native callbacks (they run before Godot's logger may be usable).</summary>
    public static readonly ConcurrentQueue<string> Messages = new();

    const string Objc = "/usr/lib/libobjc.A.dylib";
    [DllImport(Objc)] static extern IntPtr objc_getClass(string name);
    [DllImport(Objc)] static extern IntPtr sel_registerName(string name);
    [DllImport(Objc)] static extern IntPtr objc_allocateClassPair(IntPtr superclass, string name, nint extraBytes);
    [DllImport(Objc)] static extern void objc_registerClassPair(IntPtr cls);
    [DllImport(Objc)] static extern byte class_addMethod(IntPtr cls, IntPtr sel, IntPtr imp, string types);
    [DllImport(Objc)] static extern IntPtr objc_getProtocol(string name);
    [DllImport(Objc)] static extern byte class_addProtocol(IntPtr cls, IntPtr protocol);
    [DllImport(Objc, EntryPoint = "objc_msgSend")] static extern IntPtr Msg(IntPtr r, IntPtr s);
    [DllImport(Objc, EntryPoint = "objc_msgSend")] static extern IntPtr Msg(IntPtr r, IntPtr s, IntPtr a);
    [DllImport(Objc, EntryPoint = "objc_msgSend")] static extern IntPtr Msg(IntPtr r, IntPtr s, IntPtr a, IntPtr b);
    [DllImport(Objc, EntryPoint = "objc_msgSend")] static extern IntPtr MsgIndex(IntPtr r, IntPtr s, nuint i);
    [DllImport(Objc, EntryPoint = "objc_msgSend")] static extern nuint MsgCount(IntPtr r, IntPtr s);
    [DllImport(Objc, EntryPoint = "objc_msgSend")] static extern byte MsgBool(IntPtr r, IntPtr s);
    [DllImport(Objc, EntryPoint = "objc_msgSend")] static extern void MsgRect(IntPtr r, IntPtr s, CGRect rect);
    [DllImport(Objc, EntryPoint = "objc_msgSend")] static extern void MsgPresent(IntPtr r, IntPtr s, IntPtr vc, byte animated, IntPtr completion);

    [StructLayout(LayoutKind.Sequential)]
    struct CGRect { public double X, Y, W, H; }

    static IntPtr Sel(string s) => sel_registerName(s);
    static IntPtr Cls(string s) => objc_getClass(s);

    static IntPtr NSString(string s)
    {
        var p = Marshal.StringToCoTaskMemUTF8(s);
        try { return Msg(Cls("NSString"), Sel("stringWithUTF8String:"), p); }
        finally { Marshal.FreeCoTaskMem(p); }
    }

    static string FromNSString(IntPtr ns) => ns == IntPtr.Zero ? null : Marshal.PtrToStringUTF8(Msg(ns, Sel("UTF8String")));

    static bool _registered;
    public static string RegisterError { get; private set; }

    /// <summary>
    /// Runs when the assembly loads (during Godot's startup, before the window scene connects), so a launch
    /// caused by a share is not missed.
    /// </summary>
#pragma warning disable CA2255
    [ModuleInitializer]
#pragma warning restore CA2255
    internal static void Init()
    {
        if (OperatingSystem.IsIOS()) Register();
    }

    public static void Register()
    {
        if (_registered || !OperatingSystem.IsIOS()) return;
        _registered = true;
        try
        {
            var godotDelegate = Cls("GDTApplicationDelegate");
            if (godotDelegate == IntPtr.Zero) { RegisterError = "GDTApplicationDelegate not found"; return; }
            var cls = objc_allocateClassPair(Cls("NSObject"), "MMFileReceiver", 0);
            if (cls == IntPtr.Zero) { RegisterError = "could not create MMFileReceiver"; return; }
            foreach (var proto in new[] { "UIApplicationDelegate", "UISceneDelegate", "UIWindowSceneDelegate" })
                if (objc_getProtocol(proto) is var pr && pr != IntPtr.Zero) class_addProtocol(cls, pr);
            class_addMethod(cls, Sel("scene:openURLContexts:"),
                (IntPtr)(delegate* unmanaged<IntPtr, IntPtr, IntPtr, IntPtr, void>)&SceneOpenUrlContexts, "v@:@@");
            class_addMethod(cls, Sel("scene:willConnectToSession:options:"),
                (IntPtr)(delegate* unmanaged<IntPtr, IntPtr, IntPtr, IntPtr, IntPtr, void>)&SceneWillConnect, "v@:@@@");
            class_addMethod(cls, Sel("application:openURL:options:"),
                (IntPtr)(delegate* unmanaged<IntPtr, IntPtr, IntPtr, IntPtr, IntPtr, byte>)&AppOpenUrl, "c@:@@@");
            objc_registerClassPair(cls);
            var receiver = Msg(Msg(cls, Sel("alloc")), Sel("init"));
            Msg(godotDelegate, Sel("addService:"), receiver);
            Messages.Enqueue("iOS: registered to receive shared files");
        }
        catch (Exception e) { RegisterError = e.Message; }
    }

    [UnmanagedCallersOnly]
    static void SceneOpenUrlContexts(IntPtr self, IntPtr cmd, IntPtr scene, IntPtr contexts) => TakeContexts(contexts, "open");

    [UnmanagedCallersOnly]
    static void SceneWillConnect(IntPtr self, IntPtr cmd, IntPtr scene, IntPtr session, IntPtr options)
    {
        try { if (options != IntPtr.Zero) TakeContexts(Msg(options, Sel("URLContexts")), "launch"); }
        catch (Exception e) { Messages.Enqueue("iOS: launch URL error: " + e.Message); }
    }

    [UnmanagedCallersOnly]
    static byte AppOpenUrl(IntPtr self, IntPtr cmd, IntPtr app, IntPtr url, IntPtr options)
    {
        try { return TakeUrl(url, "app-open") ? (byte)1 : (byte)0; }
        catch (Exception e) { Messages.Enqueue("iOS: open URL error: " + e.Message); return 0; }
    }

    /// <summary>NSSet of UIOpenURLContext → copy each file in.</summary>
    static void TakeContexts(IntPtr set, string how)
    {
        try
        {
            if (set == IntPtr.Zero) return;
            var arr = Msg(set, Sel("allObjects"));
            nuint n = MsgCount(arr, Sel("count"));
            for (nuint i = 0; i < n; i++) TakeUrl(Msg(MsgIndex(arr, Sel("objectAtIndex:"), i), Sel("URL")), how);
        }
        catch (Exception e) { Messages.Enqueue($"iOS: {how} URL error: " + e.Message); }
    }

    static bool TakeUrl(IntPtr url, string how)
    {
        if (url == IntPtr.Zero) return false;
        var path = FromNSString(Msg(url, Sel("path")));
        Messages.Enqueue($"iOS: received ({how}) {path}");
        if (path == null) return false;
        // files outside our sandbox (opened in place from Files/iCloud) need security-scoped access while copying
        bool scoped = MsgBool(url, Sel("startAccessingSecurityScopedResource")) != 0;
        try
        {
            var dest = Import(path);
            if (dest != null) Received.Enqueue(dest);
            return dest != null;
        }
        finally { if (scoped) Msg(url, Sel("stopAccessingSecurityScopedResource")); }
    }

    static string Documents => Path.Combine(System.Environment.GetEnvironmentVariable("HOME") ?? "", "Documents");

    /// <summary>Copies (or moves, if it came via Inbox) a model into Documents/Models; returns its new path.</summary>
    static string Import(string path, bool move = false)
    {
        var ext = Path.GetExtension(path).ToLowerInvariant();
        if (ext != ".glb" && ext != ".gltf") { Messages.Enqueue($"Can't open {Path.GetFileName(path)}: ModelMate imports .glb and .gltf models only."); return null; }
        var models = Path.Combine(Documents, "Models");
        Directory.CreateDirectory(models);
        if (Path.GetDirectoryName(Path.GetFullPath(path)) == Path.GetFullPath(models)) return path;   // already ours
        var name = Path.GetFileNameWithoutExtension(path);
        var dest = Path.Combine(models, name + ext);
        for (int k = 2; File.Exists(dest); k++) dest = Path.Combine(models, $"{name} {k}{ext}");
        if (move) File.Move(path, dest); else File.Copy(path, dest);
        return dest;
    }

    /// <summary>Moves models iOS left in Documents/Inbox into Models/ and queues them.</summary>
    public static void SweepInbox()
    {
        if (!OperatingSystem.IsIOS()) return;
        try
        {
            var inbox = Path.Combine(Documents, "Inbox");
            if (!Directory.Exists(inbox)) return;
            foreach (var f in Directory.GetFiles(inbox))
            {
                Messages.Enqueue($"iOS: found {Path.GetFileName(f)} in Inbox");
                var dest = Import(f, move: true);
                if (dest != null) Received.Enqueue(dest);
            }
        }
        catch (Exception e) { Messages.Enqueue("iOS: Inbox error: " + e.Message); }
    }

    /// <summary>
    /// Presents the system share sheet for a file, anchored (on iPad, as a popover) at a rectangle in points.
    /// Must be called on the main thread. Returns false off iOS or if no view controller is available.
    /// </summary>
    public static bool Share(string filePath, float x, float y, float w, float h)
    {
        if (!OperatingSystem.IsIOS()) return false;
        var vc = Msg(Cls("GDTAppDelegateService"), Sel("viewController"));
        if (vc == IntPtr.Zero) { Messages.Enqueue("iOS: no view controller to present the share sheet"); return false; }
        var url = Msg(Cls("NSURL"), Sel("fileURLWithPath:"), NSString(filePath));
        var items = Msg(Cls("NSArray"), Sel("arrayWithObject:"), url);
        var avc = Msg(Msg(Cls("UIActivityViewController"), Sel("alloc")), Sel("initWithActivityItems:applicationActivities:"), items, IntPtr.Zero);
        var pop = Msg(avc, Sel("popoverPresentationController"));
        if (pop != IntPtr.Zero)
        {
            Msg(pop, Sel("setSourceView:"), Msg(vc, Sel("view")));
            MsgRect(pop, Sel("setSourceRect:"), new CGRect { X = x, Y = y, W = w, H = h });
        }
        MsgPresent(vc, Sel("presentViewController:animated:completion:"), avc, 1, IntPtr.Zero);
        return true;
    }
}
