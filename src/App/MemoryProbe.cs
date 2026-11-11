using System;
using System.Runtime.InteropServices;

namespace ModelMate.App;

/// <summary>
/// Process memory as the OS accounts it. On iOS and macOS this is the kernel's "physical footprint" (task_info
/// TASK_VM_INFO), the number iOS compares against its per-app limit before stopping an app. .NET's
/// Environment.WorkingSet reports 0 on iOS. On iOS, <see cref="AvailableBytes"/> is how much more the app may use.
/// </summary>
public static unsafe class MemoryProbe
{
    const string LibSystem = "/usr/lib/libSystem.dylib";
    const int TASK_VM_INFO = 22;

    [DllImport(LibSystem)] static extern int task_info(uint task, int flavor, long* info, int* count);
    [DllImport(LibSystem)] static extern nuint os_proc_available_memory();

    static uint _task;
    static bool _machOk = OperatingSystem.IsIOS() || OperatingSystem.IsMacOS() || OperatingSystem.IsMacCatalyst();

    /// <summary>Physical footprint in bytes (falls back to the working set elsewhere).</summary>
    public static long FootprintBytes()
    {
        if (_machOk)
        {
            try
            {
                if (_task == 0)
                {
                    // mach_task_self() is a macro over the exported variable mach_task_self_
                    var lib = NativeLibrary.Load(LibSystem);
                    _task = (uint)Marshal.ReadInt32(NativeLibrary.GetExport(lib, "mach_task_self_"));
                }
                var buf = stackalloc long[96];
                int count = 192;   // in 4-byte units; the kernel fills what it has and lowers count
                if (task_info(_task, TASK_VM_INFO, buf, &count) == 0 && count >= 38) return buf[18];   // phys_footprint @ byte 144
            }
            catch (Exception) { _machOk = false; }
        }
        return Environment.WorkingSet;
    }

    /// <summary>Bytes the app can still allocate before iOS stops it, or -1 where unknown (not iOS).</summary>
    public static long AvailableBytes()
    {
        if (!OperatingSystem.IsIOS()) return -1;
        try { return (long)os_proc_available_memory(); }
        catch (Exception) { return -1; }
    }
}
