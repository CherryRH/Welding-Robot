using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 快速随机树（RRT）路径规划器
/// </summary>
public static class RrtPathPlanner
{
    // ============================================================
    // 路径质量参数（规划时使用）
    // ============================================================

    /// <summary>
    /// 规划时的额外安全裕度（米）
    /// 节点到障碍的距离必须 >= 各组 WarningDistance + PlanningMargin
    /// </summary>
    private const float PlanningMargin = 0.005f;

    private const int TrajectoryWindow = 30;       // 绕圈检测窗口

    /// <summary>
    /// 路径质量配置（平滑时使用）
    /// </summary>
    public class SmoothConfig
    {
        public int MaxIterations = 100;         // 平滑最大迭代次数
        public float SegmentCheckStep = 0.005f; // 线段碰撞检测采样步长（米）
        /// <summary>平滑时的额外安全裕度，可比规划时更宽松</summary>
        public float SmoothingMargin = 0.005f;
    }

    // ============================================================
    // 树节点
    // ============================================================
    private class RrtNode
    {
        public Vector3 Position;
        public Quaternion Rotation;
        public RrtNode Parent;
        public float Cost;

        public RrtNode(Vector3 pos, Quaternion rot, RrtNode parent = null)
        {
            Position = pos;
            Rotation = rot;
            Parent = parent;
            Cost = parent != null ? parent.Cost + Vector3.Distance(pos, parent.Position) : 0f;
        }
    }

    // ============================================================
    // 规划结果
    // ============================================================
    public class PlanResult
    {
        public List<TcpPathPoint> Path;
        public bool Success;
        public int Iterations;
        public int NodesGenerated;
        public int PathNodesBeforeSmooth;
        public int PathNodesAfterSmooth;

        public PlanResult(List<TcpPathPoint> path, bool success, int iterations,
            int nodesGenerated, int beforeSmooth, int afterSmooth)
        {
            Path = path;
            Success = success;
            Iterations = iterations;
            NodesGenerated = nodesGenerated;
            PathNodesBeforeSmooth = beforeSmooth;
            PathNodesAfterSmooth = afterSmooth;
        }
    }

    // ============================================================
    // 规划入口
    // ============================================================

    /// <summary>
    /// 使用 RRT 规划从起点到终点的无碰撞路径
    /// </summary>
    public static PlanResult Plan(
        Pose start, Pose end,
        RobotModel robot,
        CollisionMonitor shadowCollisionMonitor,
        RobotBinder shadowRobotBinder,
        WeldSeam seam = null,
        int maxIterations = 2000,
        float stepSize = 0.015f,
        float goalBias = 0.1f,
        float maxDistanceToGoal = 0.015f,
        bool enableShortcutSmoothing = false)
    {
        if (start == null || end == null || MathUtil.IsPoseEqual(start, end))
            return new PlanResult(null, false, 0, 0, 0, 0);

        float totalStartDist = Vector3.Distance(start.position, end.position);
        Debug.Log($"[RRT] Starting: dist={totalStartDist:F3}m, maxIter={maxIterations}, step={stepSize:F3}, goalBias={goalBias}");

        // 初始化树
        var root = new RrtNode(start.position, start.rotation);
        var nodes = new List<RrtNode> { root };
        int nodesGenerated = 1;

        var trajectory = new List<Vector3>(TrajectoryWindow) { start.position };

        // 迭代扩展
        for (int iter = 0; iter < maxIterations; iter++)
        {
            // ---- 1. 采样 ----
            Vector3 samplePos;
            if (Random.value < goalBias)
            {
                samplePos = end.position;
            }
            else
            {
                Vector3 center = (start.position + end.position) * 0.5f;
                Vector3 extents = new Vector3(
                    Mathf.Abs(end.position.x - start.position.x) * 0.5f + 0.1f,
                    Mathf.Abs(end.position.y - start.position.y) * 0.5f + 0.1f,
                    Mathf.Abs(end.position.z - start.position.z) * 0.5f + 0.1f
                );
                samplePos = center + new Vector3(
                    Random.Range(-extents.x, extents.x),
                    Random.Range(-extents.y, extents.y),
                    Random.Range(-extents.z, extents.z)
                );
            }

            // ---- 2. 最近邻 ----
            RrtNode nearest = FindNearest(nodes, samplePos);

            // ---- 3. 扩展步长 ----
            Vector3 dir = samplePos - nearest.Position;
            float dist = dir.magnitude;
            Vector3 newPos = dist < stepSize ? samplePos : nearest.Position + (dir / dist) * stepSize;

            // ---- 4. 姿态插值 ----
            float s = 1f - Mathf.Clamp01(Vector3.Distance(newPos, end.position) / totalStartDist);
            Quaternion newRot = Quaternion.Slerp(start.rotation, end.rotation, s);

            // ---- 5. 有效性检验（含安全裕度）----
            Pose newPose = new Pose(newPos, newRot);
            if (!IsStateValid(newPose, nearest.Position, nearest.Rotation,
                robot, shadowCollisionMonitor, shadowRobotBinder, PlanningMargin))
            {
                continue;
            }

            // ---- 6. 添加节点 ----
            var newNode = new RrtNode(newPos, newRot, nearest);
            nodes.Add(newNode);
            nodesGenerated++;

            trajectory.Add(newPos);
            if (trajectory.Count > TrajectoryWindow) trajectory.RemoveAt(0);

            // ---- 7. 到达终点 ----
            if (Vector3.Distance(newPos, end.position) < maxDistanceToGoal)
            {
                var goalNode = new RrtNode(end.position, end.rotation, newNode);
                nodes.Add(goalNode);
                nodesGenerated++;

                List<TcpPathPoint> rawPath = TracePath(goalNode, seam, robot);
                int beforeSmooth = rawPath.Count;
                int afterSmooth = beforeSmooth;

                List<TcpPathPoint> finalPath = rawPath;

                // 捷径优化（可开关）
                if (enableShortcutSmoothing)
                {
                    var smoothed = ShortcutSmoothing(rawPath, robot, shadowCollisionMonitor, shadowRobotBinder, new SmoothConfig());
                    afterSmooth = smoothed.Count;
                    finalPath = smoothed;
                }

                Debug.Log($"[RRT] Reached goal at iter {iter + 1}: {nodesGenerated} nodes, " +
                    $"path {beforeSmooth} → {afterSmooth} (shortcut={(enableShortcutSmoothing ? "on" : "off")}).");

                return new PlanResult(finalPath, true, iter + 1, nodesGenerated, beforeSmooth, afterSmooth);
            }

            // ---- 8. 绕圈检测 ----
            if (trajectory.Count >= TrajectoryWindow)
            {
                Vector3 center = Vector3.zero;
                foreach (var p in trajectory) center += p;
                center /= trajectory.Count;

                float windowRadius = 0f;
                foreach (var p in trajectory)
                    windowRadius = Mathf.Max(windowRadius, Vector3.Distance(p, center));

                float distToCenter = Vector3.Distance(newPos, center);
                if (distToCenter < windowRadius * 0.3f)
                {
                    trajectory.Clear();
                }
            }
        }

        // 未到达终点：贪心补救
        Debug.LogWarning($"[RRT] Max iterations ({maxIterations}) reached, {nodesGenerated} nodes. Trying fallback.");
        RrtNode best = FindBestEffortNode(nodes, end.position);
        if (best != null && Vector3.Distance(best.Position, end.position) < maxDistanceToGoal * 5f)
        {
            var goalNode = new RrtNode(end.position, end.rotation, best);
            List<TcpPathPoint> rawPath = TracePath(goalNode, seam, robot);
            int beforeSmooth = rawPath.Count;
            int afterSmooth = beforeSmooth;
            List<TcpPathPoint> finalPath = rawPath;

            if (enableShortcutSmoothing)
            {
                var smoothed = ShortcutSmoothing(rawPath, robot, shadowCollisionMonitor, shadowRobotBinder, new SmoothConfig());
                afterSmooth = smoothed.Count;
                finalPath = smoothed;
            }

            Debug.LogWarning($"[RRT] Fallback success: best node dist={Vector3.Distance(best.Position, end.position):F3}m, " +
                $"path {beforeSmooth} → {afterSmooth}");
            return new PlanResult(finalPath, false, maxIterations, nodesGenerated, beforeSmooth, afterSmooth);
        }

        Debug.LogError($"[RRT] Complete failure: {nodesGenerated} nodes, no valid fallback.");
        return new PlanResult(null, false, maxIterations, nodesGenerated, 0, 0);
    }

    // ============================================================
    // Shortcut Smoothing：事后捷径优化
    // 遍历路径节点对 (i, j)，尝试直接连接并删去中间节点
    // ============================================================

    /// <summary>
    /// 捷径平滑：对原始 RRT 路径进行事后优化，删去可被直线安全连接的中间节点
    /// </summary>
    private static List<TcpPathPoint> ShortcutSmoothing(
        List<TcpPathPoint> rawPath,
        RobotModel robot,
        CollisionMonitor shadowCollisionMonitor,
        RobotBinder shadowRobotBinder,
        SmoothConfig config)
    {
        if (rawPath == null || rawPath.Count <= 2)
            return rawPath;

        var path = new List<TcpPathPoint>(rawPath);

        for (int iter = 0; iter < config.MaxIterations; iter++)
        {
            bool improved = false;

            // 随机选择起止索引对尝试短路
            int i = Random.Range(0, path.Count - 1);
            int j = Random.Range(i + 1, path.Count);

            // 检查 i → j 线段是否无碰撞
            if (IsSegmentCollisionFree(path[i].Pose, path[j].Pose, config.SegmentCheckStep,
                config.SmoothingMargin, robot, shadowCollisionMonitor, shadowRobotBinder))
            {
                // 删去 i 和 j 之间的所有中间节点
                path.RemoveRange(i + 1, j - i - 1);
                improved = true;
            }

            if (!improved) break;
        }

        return path;
    }

    /// <summary>
    /// 检测两点之间的直线线段是否无碰撞
    /// 在线段上均匀采样，检查每个采样点的位姿是否有效
    /// </summary>
    private static bool IsSegmentCollisionFree(
        Pose from, Pose to,
        float sampleStep,          // 采样步长（米）
        float margin,              // 安全裕度（米）
        RobotModel robot,
        CollisionMonitor shadowCollisionMonitor,
        RobotBinder shadowRobotBinder)
    {
        float totalDist = Vector3.Distance(from.position, to.position);
        if (totalDist < 1e-6f) return true;

        int sampleCount = Mathf.CeilToInt(totalDist / sampleStep);
        if (sampleCount < 2) return true;

        for (int k = 1; k < sampleCount; k++)
        {
            float t = k / (float)sampleCount;
            Vector3 pos = Vector3.Lerp(from.position, to.position, t);
            Quaternion rot = Quaternion.Slerp(from.rotation, to.rotation, t);
            Pose samplePose = new Pose(pos, rot);

            if (!IsStateValid(samplePose, from.position, from.rotation,
                robot, shadowCollisionMonitor, shadowRobotBinder, margin))
            {
                return false;
            }
        }
        return true;
    }

    // ============================================================
    // 辅助方法
    // ============================================================

    private static RrtNode FindNearest(List<RrtNode> nodes, Vector3 pos)
    {
        RrtNode nearest = nodes[0];
        float minDist = float.MaxValue;
        foreach (var node in nodes)
        {
            float d = Vector3.Distance(node.Position, pos);
            if (d < minDist) { minDist = d; nearest = node; }
        }
        return nearest;
    }

    private static RrtNode FindBestEffortNode(List<RrtNode> nodes, Vector3 goal)
    {
        RrtNode best = null;
        float bestDist = float.MaxValue;
        foreach (var node in nodes)
        {
            float d = Vector3.Distance(node.Position, goal);
            if (d < bestDist) { bestDist = d; best = node; }
        }
        return best;
    }

    private static List<TcpPathPoint> TracePath(RrtNode goalNode, WeldSeam seam, RobotModel robot)
    {
        var nodeList = new List<RrtNode>();
        for (RrtNode cur = goalNode; cur != null; cur = cur.Parent)
            nodeList.Add(cur);
        nodeList.Reverse();

        var path = new List<TcpPathPoint>();
        for (int i = 0; i < nodeList.Count; i++)
        {
            var flag = i == 0 ? TcpPathPoint.PointFlag.Start
                : i == nodeList.Count - 1 ? TcpPathPoint.PointFlag.End
                : TcpPathPoint.PointFlag.Intermediate;

            path.Add(new TcpPathPoint(
                new Pose(nodeList[i].Position, nodeList[i].Rotation),
                TcpPathPoint.PointType.Approach, flag, seam, robot.Config.TCPMaxSpeed));
        }
        return path;
    }

    /// <summary>
    /// 检验目标姿态是否有效：
    /// ① IK 可解
    /// ② 三组碰撞（Body/Gun/Self）均为 Safe
    /// ③ 各组最近距离 >= 对应 WarningDistance + margin
    /// </summary>
    private static bool IsStateValid(
        Pose pose, Vector3 prevPos, Quaternion prevRot,
        RobotModel robot,
        CollisionMonitor shadowMonitor,
        RobotBinder shadowRobotBinder,
        float margin = 0.005f)
    {
        RobotModel shadowRobot = shadowRobotBinder.Robot;

        // IK 求解
        float[] currentJoints = shadowRobot.JointAngles;
        float[] solved = robot.IK.Solve(pose, currentJoints);
        if (solved == null || solved.Length != shadowRobot.JointsCount) return false;

        bool allZero = true;
        foreach (var j in solved) { if (Mathf.Abs(j) > 1e-6f) { allZero = false; break; } }
        if (allZero) return false;

        shadowRobot.SetJointAngles(solved);
        FK.Compute(shadowRobot);
        shadowRobotBinder.Apply();
        Physics.SyncTransforms();
        shadowMonitor.Update();

        // 使用 CollisionMonitor 的统一安全检查接口
        return shadowMonitor.IsSafeForPlanning(margin);
    }
}
