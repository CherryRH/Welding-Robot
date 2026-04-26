using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 调整路径规划器
/// </summary>
public class AdjustPathPlanner
{
    private RobotModel robot;

    public void Init(RobotModel robot)
    {
        this.robot = robot;
    }

    public List<TcpPathPoint> Plan(Pose pose, WeldSeam seam = null)
    {
        List<TcpPathPoint> points = new();
        if (pose == null) 
        {
            Debug.LogWarning("[Adjust] Pose is null, returning empty.");
            return points;
        }

        Debug.Log($"[Adjust] Planning adjust path at ({pose.position.x:F3}, {pose.position.y:F3}, {pose.position.z:F3})");

        // 起点
        points.Add(new(pose, WeldStateType.Adjust, TcpPathPoint.PointFlag.Start, seam, robot.Config.TCPMaxSpeed));
        // 安全高度
        Pose safePose = robot.GetSafePose(pose);
        InsertIntermediatePoints(points, pose, safePose, seam, robot.Config.TCPMaxSpeed);
        points.Add(new(safePose, WeldStateType.Adjust, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));
        // 进入奇异状态（同位置，不插值）
        points.Add(new(safePose, WeldStateType.Adjust, TcpPathPoint.PointFlag.SingularityApproach, seam, robot.Config.TCPMaxSpeed));
        // 翻转腕部（同位置，不插值）
        points.Add(new(safePose, WeldStateType.Adjust, TcpPathPoint.PointFlag.FlipWrist, seam, robot.Config.TCPMaxSpeed));
        // 离开奇异状态（同位置，不插值）
        points.Add(new(safePose, WeldStateType.Adjust, TcpPathPoint.PointFlag.SingularityLeave, seam, robot.Config.TCPMaxSpeed));
        // 安全高度（同位置，不插值）
        points.Add(new(safePose, WeldStateType.Adjust, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));
        // 回到起始点
        InsertIntermediatePoints(points, safePose, pose, seam, robot.Config.TCPMaxSpeed);
        points.Add(new(pose, WeldStateType.Adjust, TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));

        Debug.Log($"[Adjust] Done: {points.Count} points");
        return points;
    }

    /// <summary>
    /// 在两点之间的直线路径段中按固定间隔插入中间点
    /// 仅插值位置，姿态保持与终点一致
    /// 跳过与终点重合的点
    /// </summary>
    private void InsertIntermediatePoints(List<TcpPathPoint> points, Pose from, Pose to, WeldSeam seam, float speed)
    {
        const float interval = 0.05f; // 5cm间隔
        float distance = Vector3.Distance(from.position, to.position);
        int count = Mathf.FloorToInt(distance / interval);

        for (int i = 1; i <= count; i++)
        {
            float t = i * interval / distance;
            Vector3 pos = Vector3.Lerp(from.position, to.position, t);
            Pose intermediate = new Pose(pos, to.rotation);
            // 跳过与终点重合的点
            if (MathUtil.IsPoseEqual(intermediate, to))
                continue;
            points.Add(new(intermediate, WeldStateType.Adjust, TcpPathPoint.PointFlag.Intermediate, seam, speed));
        }
    }
}