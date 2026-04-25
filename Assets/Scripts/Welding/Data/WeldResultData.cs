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
    /// 规划状态
    /// </summary>
    public WeldTaskPlanState.PlanStatus PlanStatus;

    /// <summary>
    /// 规划路径点列表
    /// </summary>
    public List<WeldPointData> Points = new();

    /// <summary>
    /// 仿真实时帧数据：时间戳 → 帧快照
    /// </summary>
    public SortedDictionary<float, WeldResultFrame> Frames = new();

    /// <summary>
    /// 记录一帧实时数据
    /// </summary>
    public void AddFrame(WeldResultFrame frame)
    {
        Frames[frame.Timestamp] = frame;
    }
}