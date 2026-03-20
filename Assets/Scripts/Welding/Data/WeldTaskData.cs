using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接任务数据
/// </summary>
[System.Serializable]
public class WeldTaskData
{
    /// <summary>
    /// 任务名称
    /// </summary>
    public string TaskName;

    /// <summary>
    /// 工件数据
    /// </summary>
    public WorkpieceData Workpiece;

    /// <summary>
    /// 焊缝数据列表
    /// </summary>
    public List<WeldSeamData> WeldSeams;
}
