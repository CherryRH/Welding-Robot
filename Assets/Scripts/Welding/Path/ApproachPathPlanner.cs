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

    /// <summary>
    /// 是否启用路径后处理平滑器（PathSmoother）
    /// 应用于 RRT 等算法生成的路径，进行拉直、平滑和排斥优化
    /// </summary>
    public bool EnablePathSmoother = true;

    /// <summary>
    /// 路径平滑器配置（仅在 EnablePathSmoother = true 时生效）
    /// </summary>
    public PathSmoother.Config SmootherConfig = new()
    {
        Iterations = 10,
        AttractStrength = 0.0f,
        PullStrength = 0.0f,
        SmoothStrength = 0.1f,
        RepulsionStrength = 0.1f,
        RepulsionRadius = 0.02f,
        Margin = 0.005f
    };

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
        ApproachStrategy strategy = ApproachStrategy.Direct,
        ReplanRecord replanRecord = null)
    {
        if (start == null || end == null || MathUtil.IsPoseEqual(start, end))
        {
            Debug.Log("[Approach] Start == End or null, returning empty path.");
            return new List<TcpPathPoint>();
        }

        Debug.Log($"[Approach] Strategy={strategy}, " +
            $"start=({start.position.x:F3},{start.position.y:F3},{start.position.z:F3}), " +
            $"end=({end.position.x:F3},{end.position.y:F3},{end.position.z:F3})");

        return strategy switch
        {
            ApproachStrategy.Safe => PlanSafePath(start, end, seam),
            ApproachStrategy.Direct => PlanDirectPath(start, end, seam),
            ApproachStrategy.RRT => PlanRrtPath(start, end, shadowRobotBinder, shadowCollisionMonitor, seam, replanRecord),
            _ => PlanSafePath(start, end, seam),
        };
    }

    /// <summary>
    /// 安全高度路径：经安全高度绕行，避免直接穿越障碍
    /// 在直线路径段中插入中间点（间隔0.05m），仅插值位置，姿态保持不变
    /// </summary>
    private List<TcpPathPoint> PlanSafePath(Pose start, Pose end, WeldSeam seam = null)
    {
        List<TcpPathPoint> points = new();
        points.Add(new(start, WeldStateType.Approach, TcpPathPoint.PointFlag.Start, seam, robot.Config.TCPMaxSpeed));

        Pose safePose = robot.GetSafePose(start);
        // 插入中间点：start → safePose
        InsertIntermediatePoints(points, start, safePose, seam, robot.Config.TCPMaxSpeed);
        points.Add(new(safePose, WeldStateType.Approach, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));

        Pose endSafe = robot.GetSafePose(end);
        // 插入中间点：safePose → endSafe
        InsertIntermediatePoints(points, safePose, endSafe, seam, robot.Config.TCPMaxSpeed);
        points.Add(new(endSafe, WeldStateType.Approach, TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));
        // 插入中间点：endSafe → end
        InsertIntermediatePoints(points, endSafe, end, seam, robot.Config.TCPMaxSpeed);
        points.Add(new(end, WeldStateType.Approach, TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));
        Debug.Log($"[Approach] Safe: {points.Count} pts (start → safe-start → safe-end → end, with intermediates)");
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
            points.Add(new(intermediate, WeldStateType.Approach, TcpPathPoint.PointFlag.Intermediate, seam, speed));
        }
    }

    private List<TcpPathPoint> PlanDirectPath(Pose start, Pose end, WeldSeam seam = null)
    {
        List<TcpPathPoint> points = new();
        points.Add(new(start, WeldStateType.Approach, TcpPathPoint.PointFlag.Start, seam, robot.Config.TCPMaxSpeed));
        points.Add(new(end, WeldStateType.Approach, TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));

        Debug.Log($"[Approach] Direct: 2 pts");
        return points;
    }

    /// <summary>
    /// RRT 路径：委托给 RrtPathPlanner
    /// </summary>
    private List<TcpPathPoint> PlanRrtPath(
        Pose start, Pose end,
        RobotBinder shadowRobotBinder,
        CollisionMonitor shadowCollisionMonitor,
        WeldSeam seam = null,
        ReplanRecord replanRecord = null)
    {
        var result = RrtPathPlanner.Plan(
            start, end, robot, shadowCollisionMonitor, shadowRobotBinder, seam,
            maxIterations: 10000,
            stepSize: 0.01f,
            goalBias: 0.1f,
            maxDistanceToGoal: 0.01f,
            enableShortcutSmoothing: false);  // 捷径优化默认关闭，与 PathSmoother 冲突

        // 记录重规划是否成功
        if (replanRecord != null)
        {
            replanRecord.IsSuccessful = result.Success;
        }

        if (result.Success)
        {
            List<TcpPathPoint> finalPath = result.Path;
            int afterSmoother = result.Path.Count;

            // 后处理平滑器（可开关）
            if (EnablePathSmoother)
            {
                var smoothed = PathSmoother.Smooth(
                    result.Path,
                    robot,
                    shadowCollisionMonitor,
                    shadowRobotBinder,
                    SmootherConfig);
                afterSmoother = smoothed.Count;
                finalPath = smoothed;
            }

            Debug.Log($"[Approach] RRT succeeded: {result.Iterations} iters, {result.NodesGenerated} nodes, " +
                $"path {result.PathNodesBeforeSmooth} → {afterSmoother} (smoother={(EnablePathSmoother ? "on" : "off")})");

            return finalPath;
        }

        Debug.LogWarning($"[Approach] RRT failed ({result.NodesGenerated} nodes generated), falling back to Safe.");
        return PlanSafePath(start, end, seam);
    }
}