using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 接近路径规划器
/// </summary>
public class ApproachPathPlanner
{
    public enum ApproachStrategy
    {
        Safe,
        Direct,
        ObstacleAvoidance
    }
    public ApproachStrategy Strategy { get; private set; } = ApproachStrategy.Direct;

    private RobotModel robot;

    public void Init(RobotModel robot)
    {
        this.robot = robot;
    }

    public List<TcpPathPoint> Plan(Pose start, Pose end, WeldSeam seam = null)
    {
        if (start == null || end == null || MathUtil.IsPoseEqual(start, end)) return new();

        switch (Strategy)
        {
            case ApproachStrategy.Safe:
                return PlanSafePath(start, end, seam);
            case ApproachStrategy.Direct:
                return PlanDirectPath(start, end, seam);
            default:
                return PlanSafePath(start, end, seam);
        }
    }

    private List<TcpPathPoint> PlanSafePath(Pose start, Pose end, WeldSeam seam = null)
    {
        List<TcpPathPoint> points = new();
        // 起点
        points.Add(new(start, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Start, seam, robot.Config.TCPMaxSpeed));
        // 安全高度，使用终点的旋转
        Pose safePose = robot.GetSafePose(start);
        safePose.rotation = end.rotation;
        points.Add(new(safePose, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));
        // 终点上方
        points.Add(new(robot.GetSafePose(end), TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));
        // 终点
        points.Add(new(end, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));

        return points;
    }

    private List<TcpPathPoint> PlanDirectPath(Pose start, Pose end, WeldSeam seam = null)
    {
        List<TcpPathPoint> points = new();
        // 起点
        points.Add(new(start, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Start, seam, robot.Config.TCPMaxSpeed));
        // 终点
        points.Add(new(end, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));

        return points;
    }
}
