using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 接近路径规划器
/// </summary>
public class ApproachPathPlanner
{
    private RobotModel robot;

    public void Init(RobotModel robot)
    {
        this.robot = robot;
    }

    public List<TcpPathPoint> Plan(Pose start, Pose end, WeldSeam seam = null)
    {
        List<TcpPathPoint> points = new();
        if (start == null || end == null || MathUtil.IsPoseClose(start, end)) return points;
        int seamId = seam == null ? 0 : seam.Id;
        // 起点
        points.Add(new(start, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Start, seamId, robot.Config.TCPMaxSpeed));
        // 安全高度，使用终点的旋转
        Pose safePose = robot.GetSafePose(start);
        safePose.rotation = end.rotation;
        points.Add(new(safePose, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Intermediate, seamId, robot.Config.TCPMaxSpeed));
        // 终点上方
        points.Add(new(robot.GetSafePose(end), TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Intermediate, seamId, robot.Config.TCPMaxSpeed));
        // 终点
        points.Add(new(end, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.End, seamId, robot.Config.TCPMaxSpeed));

        return points;
    }
}
