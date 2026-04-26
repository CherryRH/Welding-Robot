using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接结果数据：包含规划路径点和仿真过程中的实时帧数据。
/// </summary>
[System.Serializable]
public class WeldResultData
{
    /// <summary>
    /// 任务名称
    /// </summary>
    public string TaskName;

    /// <summary>
    /// 任务文件路径
    /// </summary>
    public string TaskFilePath;

    /// <summary>
    /// IK 求解方法类型
    /// </summary>
    public IK.IKMethodType IKMethodType;

    /// <summary>
    /// 关节空间插值类型
    /// </summary>
    public TrajectoryPlanner.InterpolationMethodType InterpolationMethod;

    /// <summary>
    /// 重规划的接近路径算法
    /// </summary>
    public ApproachPathPlanner.ApproachStrategy ReplanApproachStrategy;

    /// <summary>
    /// 规划状态
    /// </summary>
    public WeldTaskPlanState.PlanStatus PlanStatus;

    /// <summary>
    /// 仿真总时间（秒）
    /// </summary>
    public float TotalTime;

    /// <summary>
    /// 规划路径点列表
    /// </summary>
    public List<WeldPointData> Points = new();

    /// <summary>
    /// 仿真实时帧数据：时间戳 → 帧快照
    /// </summary>
    public SortedDictionary<float, WeldResultFrame> Frames = new();

    /// <summary>
    /// 重规划记录列表
    /// </summary>
    public List<ReplanRecord> ReplanRecords = new();

    /// <summary>
    /// 记录一帧实时数据
    /// </summary>
    public void AddFrame(WeldResultFrame frame)
    {
        Frames[frame.Timestamp] = frame;
    }

    /// <summary>
    /// 记录一次重规划
    /// </summary>
    public void AddReplanRecord(ReplanRecord record)
    {
        ReplanRecords.Add(record);
    }
}

/// <summary>
/// 重规划记录：记录每次重规划的耗时、触发时刻和机械臂状态
/// </summary>
[System.Serializable]
public class ReplanRecord
{
    /// <summary>
    /// 算法实际计算时间（毫秒）
    /// </summary>
    public float ComputationTimeMs;

    /// <summary>
    /// 触发重规划时的仿真时间戳（秒）
    /// </summary>
    public float TriggerTimestamp;

    /// <summary>
    /// 触发重规划时的 TCP 位姿
    /// </summary>
    public Pose TcpPose;

    /// <summary>
    /// 避障算法是否成功
    /// </summary>
    public bool IsSuccessful;

    public ReplanRecord(float computationTimeMs, float triggerTimestamp, Pose pose, bool isSuccessful)
    {
        ComputationTimeMs = computationTimeMs;
        TriggerTimestamp = triggerTimestamp;
        TcpPose = pose;
        IsSuccessful = isSuccessful;
    }

    public ReplanRecord() { }
}