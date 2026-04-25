using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// 焊接结果数据写入器：负责记录仿真过程中的路径数据和实时帧数据
/// </summary>
public class WeldResultDataWriter
{
    private WeldResultData resultData = new();

    /// <summary>
    /// 仿真开始时调用，重置所有数据
    /// </summary>
    public void Init(WeldTask task)
    {
        resultData = new WeldResultData
        {
            TaskName = task?.TaskName ?? "UnknownTask",
            TaskFilePath = task?.TaskFilePath ?? "",
            PlanStatus = WeldTaskPlanState.PlanStatus.Unfinished
        };
    }

    /// <summary>
    /// 记录仿真帧的实时数据
    /// </summary>
    public void RecordFrame(float timestamp, RobotModel robotModel, TrajectorySegment currentSegment = null)
    {
        // 跳过 t=0 的预触发帧（用于建立参照，不保存）
        if (timestamp <= 0f) return;

        WeldStateType segmentType = currentSegment?.Type ?? WeldStateType.Approach;
        int seamId = currentSegment?.EndPoint?.Seam?.Id ?? -1;

        var frame = new WeldResultFrame(
            timestamp,
            robotModel.TCPPosition,
            robotModel.SmoothedTcpVelocity,
            robotModel.JointAngles,
            robotModel.JointVelocities,
            robotModel.JointAccelerations,
            segmentType,
            seamId
        );
        resultData.AddFrame(frame);

        // 输出帧详细信息
        if (resultData.Frames.Count % 100 == 0)
        {
            Debug.Log(LogUtil.FormatWeldResultFrame(frame, resultData.Frames.Count));
        }
    }

    /// <summary>
    /// 每段轨迹执行完毕后，记录端点的规划路径数据
    /// </summary>
    public void RecordSegment(TrajectorySegment segment)
    {
        resultData.Points.Add(WeldPointData.FromStartPoint(segment));
        resultData.Points.Add(WeldPointData.FromEndPoint(segment));
    }

    /// <summary>
    /// 记录规划状态（仿真结束后调用）
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
    /// 存储到 JSON 文件
    /// </summary>
    public void SaveToJson()
    {
        string resultsDir = Path.Combine(Application.persistentDataPath, "Results");
        if (!Directory.Exists(resultsDir))
            Directory.CreateDirectory(resultsDir);

        string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string fileName = $"{resultData.TaskName}_{timestamp}.json";
        string filePath = Path.Combine(resultsDir, fileName);

        string json = JsonUtil.Serialize(resultData);
        File.WriteAllText(filePath, json);
        Debug.Log($"[ResultWriter] 已保存 {resultData.Frames.Count} 帧到 {filePath}");
    }
}