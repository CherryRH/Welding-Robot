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
        if (pose == null) return points;
        // 起点
        points.Add(new(pose, TcpPathPoint.PointType.Adjust, TcpPathPoint.PointFlag.Start, seam, robot.Config.TCPMaxSpeed));
        // 安全高度，开始调姿
        Pose safePose = robot.GetSafePose(pose);
        points.Add(new(safePose, TcpPathPoint.PointType.Adjust, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));
        // 进入奇异状态
        points.Add(new(safePose, TcpPathPoint.PointType.Adjust, TcpPathPoint.PointFlag.SingularityApproach, seam, robot.Config.TCPMaxSpeed));
        // 翻腕
        points.Add(new(safePose, TcpPathPoint.PointType.Adjust, TcpPathPoint.PointFlag.FlipWrist, seam, robot.Config.TCPMaxSpeed));
        // 离开奇异状态
        points.Add(new(safePose, TcpPathPoint.PointType.Adjust, TcpPathPoint.PointFlag.SingularityLeave, seam, robot.Config.TCPMaxSpeed));
        // 下降到初始高度
        points.Add(new(safePose, TcpPathPoint.PointType.Adjust, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));
        // 回到起始点
        points.Add(new(pose, TcpPathPoint.PointType.Adjust, TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));

        return points;
    }
}
