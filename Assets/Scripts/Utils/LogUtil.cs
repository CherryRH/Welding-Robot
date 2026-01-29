using System;
using System.IO;
using System.Text;
using System.Reflection;
using UnityEngine;

public static class LogUtil
{
    /// <summary>
    /// 在 Unity 控制台输出 SimulationContext 的核心信息。
    /// 如果未提供 context，会尝试使用 FindObjectOfType&lt;SimulationContext&gt;() 查找。
    /// </summary>
    public static void PrintSimulationContextDebugLog(SimulationContext context = null)
    {
        context ??= UnityEngine.Object.FindObjectOfType<SimulationContext>();
        if (context == null)
        {
            Debug.LogWarning("PrintSimulationContextDebugLog: SimulationContext not found.");
            return;
        }

        string log = BuildSimulationContextLogString(context);
        Debug.Log(log);
    }

    /// <summary>
    /// 将与控制台输出相同的日志写入磁盘：Application.persistentDataPath/Log/ 下，文件名包含时间戳。
    /// 返回写入的完整文件路径，失败返回 null。
    /// </summary>
    public static string SaveSimulationContextLogToFile(SimulationContext context = null)
    {
        context ??= UnityEngine.Object.FindObjectOfType<SimulationContext>();
        if (context == null)
        {
            Debug.LogWarning("SaveSimulationContextLogToFile: SimulationContext not found.");
            return null;
        }

        try
        {
            string log = BuildSimulationContextLogString(context);
            string folder = Path.Combine(Application.persistentDataPath, "Log");
            Directory.CreateDirectory(folder);

            string fileName = $"SimulationLog_{DateTime.Now:yyyyMMdd_HHmmss}.txt";
            string fullPath = Path.Combine(folder, fileName);

            File.WriteAllText(fullPath, log, Encoding.UTF8);
            Debug.Log($"Simulation log saved to: {fullPath}");
            return fullPath;
        }
        catch (Exception ex)
        {
            Debug.LogError($"SaveSimulationContextLogToFile error: {ex}");
            return null;
        }
    }

    /// <summary>
    /// 构建 SimulationContext 的可读日志字符串。
    /// 包括：State、Clock、RobotConfig、RobotModel、WeldTaskData。
    /// </summary>
    public static string BuildSimulationContextLogString(SimulationContext ctx)
    {
        var sb = new StringBuilder();

        sb.AppendLine("=== Simulation Context ===");
        sb.AppendLine($"Timestamp : {DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}");
        sb.AppendLine();

        // State
        sb.AppendLine("--- State ---");
        try
        {
            var sm = ctx.StateMachine;
            sb.AppendLine($"CurrentState    : {sm.CurrentState}");
            sb.AppendLine($"JointsCount     : { (ctx.RobotModel != null ? ctx.RobotModel.JointsCount.ToString() : "n/a") }");
        }
        catch (Exception ex)
        {
            sb.AppendLine($"(State read error) {ex.Message}");
        }
        sb.AppendLine();

        // Clock
        sb.AppendLine("--- Clock ---");
        try
        {
            var c = ctx.Clock;
            sb.AppendLine($"IsRunning       : {c.IsRunning}");
            sb.AppendLine($"Time            : {c.Time:F3}");
            sb.AppendLine($"FixedDeltaTime  : {c.FixedDeltaTime:F4}");
        }
        catch (Exception ex)
        {
            sb.AppendLine($"(Clock read error) {ex.Message}");
        }
        sb.AppendLine();

        // RobotConfig
        sb.AppendLine("--- RobotConfig ---");
        try
        {
            var rc = ctx.RobotConfig;
            if (rc == null)
            {
                sb.AppendLine("RobotConfig  : null");
            }
            else
            {
                sb.AppendLine($"RobotName       : {rc.RobotName}");
                sb.AppendLine($"TCPOffset       : {rc.TCPOffset.x:F3}, {rc.TCPOffset.y:F3}, {rc.TCPOffset.z:F3}");
                sb.AppendLine($"TeleopAngleV    : {rc.TeleopAngleV:F3}");
                sb.AppendLine($"TeleopTCPV      : {rc.TeleopTCPV:F3}");

                // JointsParameters：使用反射以便不依赖具体类型定义，易于扩展
                sb.AppendLine("JointsParameters :");
                var jpArray = rc.JointsParameters;
                if (jpArray == null)
                {
                    sb.AppendLine("  (null)");
                }
                else
                {
                    for (int i = 0; i < jpArray.Length; i++)
                    {
                        object jp = jpArray[i];
                        if (jp == null)
                        {
                            sb.AppendLine($"    Joint[{i}]  : null");
                            continue;
                        }

                        sb.AppendLine($"    Joint[{i}]  :");
                        var t = jp.GetType();

                        // 尝试读取公共字段/属性
                        var members = t.GetFields(BindingFlags.Public | BindingFlags.Instance);
                        foreach (var f in members)
                        {
                            try
                            {
                                var v = f.GetValue(jp);
                                sb.AppendLine($"        {f.Name} : {FormatObject(v)}");
                            }
                            catch { }
                        }

                        var props = t.GetProperties(BindingFlags.Public | BindingFlags.Instance);
                        foreach (var p in props)
                        {
                            if (!p.CanRead) continue;
                            try
                            {
                                var v = p.GetValue(jp, null);
                                sb.AppendLine($"        {p.Name} : {FormatObject(v)}");
                            }
                            catch { }
                        }
                    }
                }
            }
        }
        catch (Exception ex)
        {
            sb.AppendLine($"(RobotConfig read error) {ex.Message}");
        }
        sb.AppendLine();

        // RobotModel
        sb.AppendLine("--- RobotModel ---");
        try
        {
            var rm = ctx.RobotModel;
            if (rm == null)
            {
                sb.AppendLine("RobotModel   : null");
            }
            else
            {
                sb.AppendLine($"TCPPosition (MDH)           : {rm.TCPPosition.x:F3}, {rm.TCPPosition.y:F3}, {rm.TCPPosition.z:F3}");
                var euler = rm.TCPRotation.eulerAngles;
                sb.AppendLine($"TCPRotationEuler (MDH)      : {euler.x:F1}, {euler.y:F1}, {euler.z:F1}");
                sb.AppendLine($"ToolEularAngles (MDH)       : {rm.ToolEularAngles.x:F1}, {rm.ToolEularAngles.y:F1}, {rm.ToolEularAngles.z:F1}");

                sb.AppendLine($"UTCPPosition (Unity)        : {rm.UTCPPosition.x:F3}, {rm.UTCPPosition.y:F3}, {rm.UTCPPosition.z:F3}");
                var uEuler = rm.UTCPRotation.eulerAngles;
                sb.AppendLine($"UTCPRotationEuler (Unity)   : {uEuler.x:F1}, {uEuler.y:F1}, {uEuler.z:F1}");

                var joints = rm.Joints;
                if (joints == null)
                {
                    sb.AppendLine("Joints   : null");
                }
                else
                {
                    sb.AppendLine("Joints   :");
                    for (int i = 0; i < joints.Length; i++)
                    {
                        var j = joints[i];
                        if (j == null)
                        {
                            sb.AppendLine($"    Joint[{i}]  : null");
                            continue;
                        }

                        sb.AppendLine($"    Joint[{i}]    :");
                        sb.AppendLine($"        Angle       : {j.Angle:F3} deg");
                        sb.AppendLine($"        AngleV      : {j.AngleV:F3} deg/s");
                        sb.AppendLine($"        AngleA      : {j.AngleA:F3} deg/s²");
                        var pos = j.Position;
                        sb.AppendLine($"        Position (MDH)      : {pos.x:F3}, {pos.y:F3}, {pos.z:F3}");
                        var upos = j.UPosition;
                        sb.AppendLine($"        UPosition (Unity)   : {upos.x:F3}, {upos.y:F3}, {upos.z:F3}");

                        // 输出矩阵（局部 / 世界）简洁表示（每行）
                        sb.AppendLine("        LocalTransform:");
                        sb.AppendLine(FormatMatrix(j.LocalTransform, "            "));
                        sb.AppendLine("        WorldTransform:");
                        sb.AppendLine(FormatMatrix(j.WorldTransform, "            "));
                    }
                }
            }
        }
        catch (Exception ex)
        {
            sb.AppendLine($"(RobotModel read error) {ex.Message}");
        }
        sb.AppendLine();

        // WeldTaskData
        sb.AppendLine("--- WeldTaskData ---");
        try
        {
            var wtd = ctx.WeldTaskData;
            if (wtd == null)
            {
                sb.AppendLine("WeldTaskData   : null");
            }
            else
            {
                sb.AppendLine($"TaskName            : {wtd.TaskName}");
                sb.AppendLine($"WorkpieceFileName   : {wtd.WorkpieceFileName}");
                sb.AppendLine($"WeldSeamsCount      : {(wtd.WeldSeams != null ? wtd.WeldSeams.Count.ToString() : "n/a") }");
                if (wtd.WeldSeams != null)
                {
                    for (int i = 0; i < wtd.WeldSeams.Count; i++)
                    {
                        var seg = wtd.WeldSeams[i];
                        if (seg == null)
                        {
                            sb.AppendLine($"    WeldSeams[{i}]  : null");
                            continue;
                        }
                        sb.AppendLine($"    WeldSeams[{i}]  :");
                        sb.AppendLine($"        Type            : {seg.Type}");
                        sb.AppendLine($"        ID              : {seg.ID}");
                        sb.AppendLine($"        Name            : {seg.Name}");
                        sb.AppendLine($"        Speed           : {seg.Speed:F3} m/s");
                        sb.AppendLine($"        GunAngle        : {seg.GunAngle:F3} deg");
                        sb.AppendLine($"        GunDistance     : {seg.GunDistance:F3} m");
                        sb.AppendLine($"        ConnectionRelationship : {seg.ConnectionRelationship}");
                        sb.AppendLine($"        StartPoint      : {FormatObject(seg.StartPoint)}");
                        sb.AppendLine($"        EndPoint        : {FormatObject(seg.EndPoint)}");
                        if (seg.MiddlePoints != null)
                        {
                            for (int j = 0; j < seg.MiddlePoints.Count; j++)
                            {
                                sb.AppendLine($"        MiddlePoint[{j}]    : {FormatObject(seg.MiddlePoints[j])}");
                            }
                        }
                        sb.AppendLine($"        Normal          : {FormatObject(seg.Normal)}");
                        sb.AppendLine($"        LengthDeviation : {seg.LengthDeviation:F3} m");
                    }
                }
            }
        }
        catch (Exception ex)
        {
            sb.AppendLine($"(WeldTaskData read error) {ex.Message}");
        }
        sb.AppendLine();

        sb.AppendLine("=== End ===");

        return sb.ToString();
    }

    // 简单格式化通用对象
    private static string FormatObject(object o)
    {
        if (o == null) return "null";
        if (o is float f) return f.ToString("F3");
        if (o is double d) return d.ToString("F3");
        if (o is Vector3 v) return $"{v.x:F3}, {v.y:F3}, {v.z:F3}";
        if (o is Quaternion q) return $"{q.eulerAngles.x:F1}, {q.eulerAngles.y:F1}, {q.eulerAngles.z:F1}";
        if (o is Matrix4x4 m) return FormatMatrixInline(m);
        return o.ToString();
    }

    // 单行矩阵简洁表示
    private static string FormatMatrixInline(Matrix4x4 m)
    {
        // 行优先，简短显示前三列与最后列
        return string.Format(
            "[{0:F3},{1:F3},{2:F3},{3:F3}; {4:F3},{5:F3},{6:F3},{7:F3}; {8:F3},{9:F3},{10:F3},{11:F3}; {12:F3},{13:F3},{14:F3},{15:F3}]",
            m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33
        );
    }

    // 多行矩阵格式，带前缀
    private static string FormatMatrix(Matrix4x4 m, string prefix)
    {
        var sb = new StringBuilder();
        sb.AppendLine($"{prefix}[{m.m00:F3}, {m.m01:F3}, {m.m02:F3}, {m.m03:F3}]");
        sb.AppendLine($"{prefix}[{m.m10:F3}, {m.m11:F3}, {m.m12:F3}, {m.m13:F3}]");
        sb.AppendLine($"{prefix}[{m.m20:F3}, {m.m21:F3}, {m.m22:F3}, {m.m23:F3}]");
        sb.AppendLine($"{prefix}[{m.m30:F3}, {m.m31:F3}, {m.m32:F3}, {m.m33:F3}]");
        return sb.ToString();
    }
}
