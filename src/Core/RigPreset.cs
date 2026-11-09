using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using Godot;

namespace ModelMate.Core;

/// <summary>
/// Skeleton presets. "humanoid" and "single" are fitted by code (HumanoidTemplate); every other preset is a JSON
/// file whose joint positions are fractions of the model's bounding box, so one preset refits to any model.
/// Built-ins live in res://rigs, user presets in user://rigs.
/// </summary>
public static class RigPreset
{
    public sealed class BoneDto
    {
        [JsonPropertyName("name")] public string Name { get; set; }
        [JsonPropertyName("parent")] public string Parent { get; set; }
        [JsonPropertyName("head")] public float[] Head { get; set; }
        [JsonPropertyName("tail")] public float[] Tail { get; set; }
        [JsonPropertyName("tailChild")] public string TailChild { get; set; }
        [JsonPropertyName("deform")] public bool Deform { get; set; } = true;
        [JsonPropertyName("center")] public string Center { get; set; }
    }

    public sealed class PoseRotDto
    {
        [JsonPropertyName("bone")] public string Bone { get; set; }
        [JsonPropertyName("axis")] public float[] Axis { get; set; }   // bone-local axis
        [JsonPropertyName("deg")] public float Deg { get; set; }
    }

    public sealed class PresetDto
    {
        [JsonPropertyName("name")] public string Name { get; set; }
        [JsonPropertyName("description")] public string Description { get; set; }
        /// <summary>Center joints in the model's volume after fitting. Built-ins: true (adapt to a new model);
        /// saved rigs: false (reload hand-placed joints exactly).</summary>
        [JsonPropertyName("autoCenter")] public bool AutoCenter { get; set; } = true;
        [JsonPropertyName("bones")] public List<BoneDto> Bones { get; set; } = new();
        [JsonPropertyName("poses")] public Dictionary<string, List<PoseRotDto>> Poses { get; set; } = new();
    }

    static readonly JsonSerializerOptions Json = new()
    {
        WriteIndented = true,
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
    };

    public const string BuiltinDir = "res://rigs";
    public const string UserDir = "user://rigs";

    /// <summary>Preset names: code presets, then built-in JSON, then user JSON.</summary>
    public static List<string> List()
    {
        var names = new List<string> { "humanoid", "single" };
        foreach (var dir in new[] { BuiltinDir, UserDir })
        {
            var abs = ProjectSettings.GlobalizePath(dir);
            if (!Directory.Exists(abs)) continue;
            foreach (var f in Directory.GetFiles(abs, "*.json").OrderBy(x => x))
            {
                var n = Path.GetFileNameWithoutExtension(f);
                if (!names.Contains(n)) names.Add(n);
            }
        }
        return names;
    }

    /// <summary>Resolves a preset name or a .json path to a file path (null for code presets or unknown names).</summary>
    public static string Resolve(string nameOrPath)
    {
        if (nameOrPath.EndsWith(".json", StringComparison.OrdinalIgnoreCase))
            return ProjectSettings.GlobalizePath(nameOrPath);
        foreach (var dir in new[] { UserDir, BuiltinDir })
        {
            var p = Path.Combine(ProjectSettings.GlobalizePath(dir), nameOrPath + ".json");
            if (File.Exists(p)) return p;
        }
        return null;
    }

    public static PresetDto Load(string path) =>
        JsonSerializer.Deserialize<PresetDto>(File.ReadAllText(path), Json) ?? throw new Exception($"empty preset {path}");

    /// <summary>Builds and fits a skeleton for the model bounds. Bones with a center mode are centered in the volume.</summary>
    public static RigSkeleton Fit(string nameOrPath, Aabb box, Bvh bvh, Action<string> log, out PresetDto preset)
    {
        preset = null;
        if (nameOrPath == "humanoid") return HumanoidTemplate.Fit(box, bvh, log);
        if (nameOrPath == "single") return HumanoidTemplate.Single(box);
        var path = Resolve(nameOrPath) ?? throw new Exception($"Unknown rig preset '{nameOrPath}'. Available: {string.Join(", ", List())}");
        preset = Load(path);
        var s = FromDto(preset, box);
        if (bvh != null && preset.AutoCenter && s.Bones.Any(b => b.Center != CenterMode.None))
            HumanoidTemplate.CenterInVolume(s, bvh, box.Size.Y, log);
        var errs = RigEdit.Validate(s);
        if (errs.Count > 0) throw new Exception($"Preset {nameOrPath} is invalid: {string.Join("; ", errs)}");
        log?.Invoke($"Rig preset '{preset.Name ?? nameOrPath}' ({s.Bones.Count} bones) from {path}");
        return s;
    }

    static Vector3 FromFrac(float[] f, Aabb box) => box.Position + new Vector3(f[0], f[1], f[2]) * box.Size;

    static float[] ToFrac(Vector3 p, Aabb box)
    {
        var sz = box.Size;
        float F(float v, float o, float s) => s > 1e-9f ? (v - o) / s : 0.5f;
        return new[] { MathF.Round(F(p.X, box.Position.X, sz.X), 5), MathF.Round(F(p.Y, box.Position.Y, sz.Y), 5), MathF.Round(F(p.Z, box.Position.Z, sz.Z), 5) };
    }

    public static RigSkeleton FromDto(PresetDto dto, Aabb box)
    {
        var s = new RigSkeleton();
        foreach (var b in dto.Bones)
            s.Bones.Add(new RigBone
            {
                Name = b.Name, Head = FromFrac(b.Head, box), TailEnd = FromFrac(b.Tail ?? b.Head, box), Deform = b.Deform,
                Center = Enum.TryParse<CenterMode>(b.Center ?? "None", out var cm) ? cm : CenterMode.None,
            });
        for (int i = 0; i < dto.Bones.Count; i++)
        {
            var d = dto.Bones[i];
            s.Bones[i].Parent = d.Parent != null ? s.Find(d.Parent) : -1;
            if (d.Parent != null && s.Bones[i].Parent < 0) throw new Exception($"bone {d.Name}: unknown parent {d.Parent}");
        }
        for (int i = 0; i < dto.Bones.Count; i++)
            if (dto.Bones[i].TailChild != null) s.Bones[i].PrimaryChild = s.Find(dto.Bones[i].TailChild);
        RigEdit.Normalize(s);
        return s;
    }

    public static PresetDto ToDto(RigSkeleton s, Aabb box, string name, string description = null,
        Dictionary<string, List<PoseRotDto>> poses = null)
    {
        var dto = new PresetDto { Name = name, Description = description, Poses = poses ?? new(), AutoCenter = false };
        for (int i = 0; i < s.Bones.Count; i++)
        {
            var b = s.Bones[i];
            dto.Bones.Add(new BoneDto
            {
                Name = b.Name, Parent = b.Parent >= 0 ? s.Bones[b.Parent].Name : null,
                Head = ToFrac(b.Head, box), Tail = ToFrac(s.Tail(i), box),
                TailChild = b.PrimaryChild >= 0 ? s.Bones[b.PrimaryChild].Name : null,
                Deform = b.Deform, Center = b.Center != CenterMode.None ? b.Center.ToString() : null,
            });
        }
        return dto;
    }

    /// <summary>Local rotation per bone name for a named pose (bone-local axes, so it applies to any fitted rig).</summary>
    public static Dictionary<string, Basis> PoseRotations(PresetDto dto, string pose)
    {
        if (dto?.Poses == null || !dto.Poses.TryGetValue(pose, out var rots))
            throw new Exception($"Pose '{pose}' not found. Available: {string.Join(", ", dto?.Poses?.Keys ?? Enumerable.Empty<string>())}");
        var d = new Dictionary<string, Basis>();
        foreach (var r in rots)
        {
            var b = new Basis(new Vector3(r.Axis[0], r.Axis[1], r.Axis[2]).Normalized(), Mathf.DegToRad(r.Deg));
            d[r.Bone] = d.TryGetValue(r.Bone, out var prev) ? prev * b : b;
        }
        return d;
    }

    /// <summary>Saves to a path, or to user://rigs/&lt;name&gt;.json when path is null. Returns the absolute path.</summary>
    public static string Save(RigSkeleton s, Aabb box, string name, string path = null, string description = null,
        Dictionary<string, List<PoseRotDto>> poses = null)
    {
        path ??= Path.Combine(ProjectSettings.GlobalizePath(UserDir), name + ".json");
        Directory.CreateDirectory(Path.GetDirectoryName(path));
        File.WriteAllText(path, JsonSerializer.Serialize(ToDto(s, box, name, description, poses), Json));
        return path;
    }
}
