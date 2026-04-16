using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接点数据，储存焊接仿真数据点，包含TcpPathPoint部分数据以及对应的TrajectorySegment部分数据，作为焊接仿真结果的基本单元
/// </summary>
public class WeldPointData
{
    // ========== 控制逻辑数据（来自 TcpPathPoint）==========

    /// <summary>
    /// TCP 位姿（Data 坐标系）
    /// </summary>
    public Pose TcpPose;

    /// <summary>
    /// 所属焊缝 ID
    /// </summary>
    public int SeamId;

    /// <summary>
    /// 焊缝名称
    /// </summary>
    public string SeamName;

    /// <summary>
    /// 路径点类型（Approach / Weld / Adjust）
    /// </summary>
    public WeldStateType Type;

    /// <summary>
    /// 路径点标记（Start / End / Intermediate 等）
    /// </summary>
    public TcpPathPoint.PointFlag PathFlag;

    // ========== 实时姿态数据（来自 TrajectorySegment）==========

    /// <summary>
    /// 对应时刻（仿真时间，秒）
    /// </summary>
    public float Time;

    /// <summary>
    /// 关节角（度），长度 = 关节数
    /// </summary>
    public float[] JointAngles;

    /// <summary>
    /// 关节角速度（度/秒），长度 = 关节数
    /// </summary>
    public float[] JointVelocities;

    /// <summary>
    /// 关节角加速度（度/秒²），长度 = 关节数
    /// </summary>
    public float[] JointAccelerations;

    /// <summary>
    /// 轨迹点类型（Approach / Weld / Adjust）
    /// </summary>
    public WeldStateType PointType;

    /// <summary>
    /// 由段起点构造（记录进入该段时的状态）
    /// </summary>
    public static WeldPointData FromStartPoint(TrajectorySegment segment)
    {
        return new WeldPointData
        {
            TcpPose = segment.StartPoint.Pose,
            SeamId = segment.StartPoint.Seam?.Id ?? -1,
            SeamName = segment.StartPoint.Seam?.Name ?? "",
            Type = segment.StartPoint.Type,
            PathFlag = segment.StartPoint.Flag,
            Time = segment.StartTime,
            JointAngles = segment.QStart,
            JointVelocities = segment.Interpolation?.EvaluateVelocity(0f),
            JointAccelerations = segment.Interpolation?.EvaluateAcceleration(0f),
            PointType = segment.Type
        };
    }

    /// <summary>
    /// 由段终点构造（记录该段结束时的状态）
    /// </summary>
    public static WeldPointData FromEndPoint(TrajectorySegment segment)
    {
        float duration = segment.EndTime - segment.StartTime;
        return new WeldPointData
        {
            TcpPose = segment.EndPoint.Pose,
            SeamId = segment.EndPoint.Seam?.Id ?? -1,
            SeamName = segment.EndPoint.Seam?.Name ?? "",
            Type = segment.EndPoint.Type,
            PathFlag = segment.EndPoint.Flag,
            Time = segment.EndTime,
            JointAngles = segment.QEnd,
            JointVelocities = segment.Interpolation?.EvaluateVelocity(duration),
            JointAccelerations = segment.Interpolation?.EvaluateAcceleration(duration),
            PointType = segment.Type
        };
    }
}
