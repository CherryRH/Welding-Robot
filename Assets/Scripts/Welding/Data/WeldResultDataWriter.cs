using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// 焊接结果数据写入器：管理数据记录与文件持久化
/// </summary>
public class WeldResultDataWriter
{
    private WeldResultData resultData = new();

    /// <summary>
    /// 任务开始时调用，设置任务名
    /// </summary>
    public void Init(string taskName)
    {
        resultData = new WeldResultData
        {
            TaskName = taskName,
            PlanStatus = WeldTaskPlanState.PlanStatus.Unfinished
        };
    }

    /// <summary>
    /// 每个轨迹段执行完毕后调用，记录该段的起点和终点数据
    /// </summary>
    public void RecordSegment(TrajectorySegment segment)
    {
        resultData.Points.Add(WeldPointData.FromStartPoint(segment));
        resultData.Points.Add(WeldPointData.FromEndPoint(segment));
    }

    /// <summary>
    /// 任务结束时调用，设置最终状态
    /// </summary>
    public void SetPlanStatus(WeldTaskPlanState.PlanStatus status)
    {
        resultData.PlanStatus = status;
    }

    /// <summary>
    /// 获取当前结果数据
    /// </summary>
    public WeldResultData GetResultData() => resultData;

    /// <summary>
    /// 保存到 JSON 文件（持久化路径/Results/）
    /// </summary>
    public void SaveToJson()
    {
        string resultsDir = Path.Combine(Application.persistentDataPath, "Results");
        if (!Directory.Exists(resultsDir))
            Directory.CreateDirectory(resultsDir);

        // 文件名：任务名_时间戳.json
        string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string fileName = $"{resultData.TaskName}_{timestamp}.json";
        string filePath = Path.Combine(resultsDir, fileName);

        string json = JsonUtil.Serialize(resultData);
        File.WriteAllText(filePath, json);
        Debug.Log($"[WeldResultDataWriter] 已保存焊接结果：{filePath}");
    }
}
