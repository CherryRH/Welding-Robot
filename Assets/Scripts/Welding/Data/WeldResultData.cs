using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接结果数据，包含任务结果信息和所有焊接点数据列表
/// </summary>
public class WeldResultData
{
    /// <summary>
    /// 任务名称
    /// </summary>
    public string TaskName;

    /// <summary>
    /// 规划状态
    /// </summary>
    public WeldTaskPlanState.PlanStatus PlanStatus;

    /// <summary>
    /// 焊接点数据列表
    /// </summary>
    public List<WeldPointData> Points = new();
}
