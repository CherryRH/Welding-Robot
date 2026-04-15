using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 接近路径规划器
/// </summary>
public class ApproachPathPlanner
{
    /// <summary>
    /// 接近策略枚举
    /// </summary>
    public enum ApproachStrategy
    {
        Safe,   // 经安全高度（Z轴上方）绕行
        Direct, // 直线插值
        RRT     // 快速随机树（概率完备，不易失败）
    }

    private RobotModel robot;

    public void Init(RobotModel robot)
    {
        this.robot = robot;
    }

    /// <summary>
    /// 规划接近路径（显式指定策略，供重规划调用）
    /// </summary>
    /// <param name="start">起点（Data 坐标系）</param>
    /// <param name="end">终点（Data 坐标系）</param>
    /// <param name="shadowCollisionMonitor">影子机械臂碰撞监测器</param>
    /// <param name="shadowRobotBinder">影子机械臂绑定器</param>
    /// <param name="seam">焊缝信息</param>
    /// <param name="strategy">使用的接近策略</param>
    public List<TcpPathPoint> Plan(
        Pose start, Pose end,
        CollisionMonitor shadowCollisionMonitor,
        RobotBinder shadowRobotBinder,
        WeldSeam seam = null,
        ApproachStrategy strategy = ApproachStrategy.Direct)
    {
        if (start == null || end == null || MathUtil.IsPoseEqual(start, end))
            return new List<TcpPathPoint>();

        return strategy switch
        {
            ApproachStrategy.Safe => PlanSafePath(start, end, seam),
            ApproachStrategy.Direct => PlanDirectPath(start, end, seam),
            ApproachStrategy.RRT => PlanRrtPath(start, end, shadowRobotBinder, shadowCollisionMonitor, seam),
            _ => PlanSafePath(start, end, seam),
        };
    }

    /// <summary>
    /// 安全高度路径：经安全高度绕行，避免直接穿越障碍
    /// </summary>
    private List<TcpPathPoint> PlanSafePath(Pose start, Pose end, WeldSeam seam = null)
    {
        List<TcpPathPoint> points = new();
        points.Add(new(start, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Start, seam, robot.Config.TCPMaxSpeed));

        Pose safePose = robot.GetSafePose(start);
        safePose.rotation = end.rotation;
        points.Add(new(safePose, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));

        Pose endSafe = robot.GetSafePose(end);
        points.Add(new(endSafe, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));
        points.Add(new(end, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));
        return points;
    }

    /// <summary>
    /// 直线路径：起点 + 终点，无中间点
    /// </summary>
    private List<TcpPathPoint> PlanDirectPath(Pose start, Pose end, WeldSeam seam = null)
    {
        List<TcpPathPoint> points = new();
        points.Add(new(start, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.Start, seam, robot.Config.TCPMaxSpeed));
        points.Add(new(end, TcpPathPoint.PointType.Approach, TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));
        return points;
    }

    /// <summary>
    /// RRT 路径：委托给 RrtPathPlanner
    /// </summary>
    private List<TcpPathPoint> PlanRrtPath(
        Pose start, Pose end,
        RobotBinder shadowRobotBinder,
        CollisionMonitor shadowCollisionMonitor,
        WeldSeam seam = null)
    {
        var result = RrtPathPlanner.Plan(
            start, end, robot, shadowCollisionMonitor, shadowRobotBinder, seam,
            maxIterations: 2000,
            stepSize: 0.015f,
            goalBias: 0.1f,
            maxDistanceToGoal: 0.015f);

        if (result.Success)
        {
            Debug.Log($"[ApproachPathPlanner] RRT succeeded: {result.Iterations} iters, {result.NodesGenerated} nodes, " +
                $"path {result.PathNodesBeforeSmooth} → {result.PathNodesAfterSmooth} after smoothing.");
            return result.Path;
        }

        Debug.LogWarning($"[ApproachPathPlanner] RRT failed ({result.NodesGenerated} nodes generated), falling back to Safe.");
        return PlanSafePath(start, end, seam);
    }
}
