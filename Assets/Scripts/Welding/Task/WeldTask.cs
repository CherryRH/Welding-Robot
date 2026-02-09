using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接任务
/// </summary>
public class WeldTask
{
    /// <summary>
    /// 任务名称
    /// </summary>
    public string TaskName;

    /// <summary>
    /// 工件模型文件名称
    /// </summary>
    public string WorkpieceFileName;

    /// <summary>
    /// 焊缝列表
    /// </summary>
    public List<WeldSeam> WeldSeams = new();

    public WeldTask(WeldTaskData data)
    {
        // 构建焊接任务
        TaskName = data.TaskName;
        WorkpieceFileName = data.WorkpieceFileName;
        foreach (var seamData in data.WeldSeams)
        {
            WeldSeam seam = seamData.Type switch
            {
                WeldSeamData.WeldSeamType.Line => new LineSeam(seamData),
                WeldSeamData.WeldSeamType.Arc => new ArcSeam(seamData),
                _ => null
            };
            if (seam != null)
            {
                WeldSeams.Add(seam);
            }
        }
    }

    /// <summary>
    /// 优化焊缝执行顺序，最小化总路径长度
    /// </summary>
    public void Optimize()
    {

    }
}
