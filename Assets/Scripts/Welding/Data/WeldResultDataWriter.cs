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
    public void Init(SimulationContext ctx)
    {
        resultData = new WeldResultData
        {
            TaskName = ctx.Task?.TaskName ?? "UnknownTask",
            TaskFilePath = ctx.Task?.TaskFilePath ?? "",
            PlanStatus = WeldTaskPlanState.PlanStatus.Unfinished,
            ReplanApproachStrategy = ctx.TcpPathPlanner.ReplanApproachStrategy,
            IKMethodType = ctx.RobotModel.IK.IKMethod,
            InterpolationMethod = ctx.TrajectoryPlanner.InterpolationMethod
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
        WeldSeam seam = currentSegment?.StartPoint?.Seam;
        int seamId = seam?.Id ?? -1;

        // --- 计算误差 ---
        float speedError = 0f;
        float positionError = 0f;
        float orientationError = 0f;

        // 1. 线速度误差：当前速度 - 参考速度（取起点参考速度）
        float refSpeed = currentSegment?.StartPoint?.Speed ?? 0f;
        speedError = robotModel.TcpSpeed - refSpeed;

        // 2. 焊点误差和姿态误差
        if (seam != null)
        {
            // 当前焊枪方向：TCP的forward（Data坐标系）
            Vector3 currentGunDir = robotModel.TCPRotation * Vector3.forward;

            // 焊点误差：TCP位置转换到Data坐标系后计算
            Vector3 tcpPosInData = robotModel.RobotToUser(robotModel.TCPPosition);
            positionError = seam.ComputePositionError(tcpPosInData + currentGunDir * seam.GunDistance);

            // 焊枪姿态误差
            // 参考方向：基于当前TCP位置在焊缝上的投影点计算
            Vector3 weldDir = seam.GetTangent(seam.ProjectPointToSeam(tcpPosInData));
            Vector3 refGunDir = seam.ComputeGunDirection(weldDir);

            // 计算夹角（度）
            orientationError = Vector3.Angle(currentGunDir, refGunDir);
        }

        var frame = new WeldResultFrame(
            timestamp,
            robotModel.TCPPosition,
            robotModel.SmoothedTcpVelocity,
            robotModel.JointAngles,
            robotModel.JointVelocities,
            robotModel.JointAccelerations,
            segmentType,
            seamId,
            speedError,
            positionError,
            orientationError
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
    /// 记录结果状态（仿真结束后调用）
    /// </summary>
    public void SetPlanStatus(WeldTaskPlanState state, float totalTime)
    {
        resultData.PlanStatus = state.Status;
        resultData.TotalTime = totalTime;
    }

    /// <summary>
    /// 记录一次重规划
    /// </summary>
    public void AddReplanRecord(ReplanRecord record)
    {
        resultData?.AddReplanRecord(record);
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