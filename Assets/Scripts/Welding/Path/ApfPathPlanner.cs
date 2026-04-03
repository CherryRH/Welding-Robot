using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 人工势场法（APF）路径规划器
/// 
/// 算法特性：
/// - 梯度下降类方法，速度快但易陷入局部极小
/// - 法向排斥 + 切向滑动混合，实现沿障碍 wall-following 绕行
/// - 锁定绕行方向防止抖动，距离衰减使能离开墙体
/// 
/// 适用于：障碍简单、路径可通的场景
/// 不适用于：复杂多障碍、狭窄通道（建议用 RRT）
/// </summary>
public static class ApfPathPlanner
{
    /// <summary>
    /// 使用 APF 规划从起点到终点的无碰撞路径
    /// </summary>
    /// <param name="start">起点（Data 坐标系）</param>
    /// <param name="end">终点（Data 坐标系）</param>
    /// <param name="robot">机械臂模型（用于 IK 求解）</param>
    /// <param name="shadowCollisionMonitor">影子机械臂碰撞监测器</param>
    /// <param name="shadowRobotBinder">影子机械臂绑定器（用于 Apply Transform）</param>
    /// <param name="seam">焊缝信息（用于标记路径点，可为 null）</param>
    /// <returns>路径点列表，失败时返回 null</returns>
    public static List<TcpPathPoint> Plan(
        Pose start, Pose end,
        RobotModel robot,
        CollisionMonitor shadowCollisionMonitor,
        RobotBinder shadowRobotBinder,
        WeldSeam seam = null)
    {
        if (start == null || end == null || MathUtil.IsPoseEqual(start, end))
            return null;

        List<TcpPathPoint> points = new();

        // ============================================================
        // 参数
        // ============================================================
        const int maxIterations = 500;
        const int maxConsecutiveSkips = 30;
        const float stepSize = 0.01f;         // 米，Data 坐标系
        const float influenceRadius = 0.1f;   // 米
        const float tolerance = 0.005f;      // 米
        const float tangentWeight = 0.8f;    // 切向力权重
        const float normalWeight = 0.3f;     // 法向力权重
        const int trajectoryWindow = 30;    // 轨迹震荡检测窗口

        // ============================================================
        // 状态
        // ============================================================
        Pose current = start;
        int consecutiveSkips = 0;
        float totalStartDist = Vector3.Distance(start.position, end.position);
        var trajectory = new List<Vector3>(trajectoryWindow);
        bool wallFollowLocked = false;
        int wallFollowSign = 0;

        for (int i = 0; i < maxIterations; i++)
        {
            Vector3 pos = current.position;
            Vector3 goal = end.position;

            // ---- 1. 吸引力 ----
            Vector3 dirToGoal = goal - pos;
            float distToGoal = dirToGoal.magnitude;
            Vector3 F_att = distToGoal > 1e-6f ? dirToGoal.normalized : Vector3.zero;

            // ---- 2. 检测"可直接朝目标走"，解除贴墙锁定 ----
            if (wallFollowLocked && CanGoDirect(pos, goal, shadowCollisionMonitor))
            {
                wallFollowLocked = false;
                wallFollowSign = 0;
            }

            // ---- 3. 合力计算 ----
            Vector3 F = F_att;
            Vector3 posU = MathUtil.D2UPosition(pos);
            float envDist = shadowCollisionMonitor.DistancePointToEnvironment(posU, out Vector3 closestU);
            Vector3 closest = MathUtil.U2DPosition(closestU);

            if (envDist < influenceRadius && envDist > 1e-6f)
            {
                Vector3 normal = (pos - closest).normalized;

                // 切线候选：normal × up 的两个半轴
                Vector3 tangent1 = Vector3.Cross(normal, Vector3.up);
                if (tangent1.sqrMagnitude < 1e-6f)
                    tangent1 = Vector3.Cross(normal, Vector3.forward);
                Vector3 tangent2 = -tangent1;

                // 第一次进入障碍 → 锁定绕行方向
                if (!wallFollowLocked)
                {
                    wallFollowLocked = true;
                    wallFollowSign = Vector3.Dot(tangent1, F_att) > 0 ? 1 : -1;
                }

                Vector3 tangent = (wallFollowSign > 0 ? tangent1 : tangent2).normalized;

                // 距离衰减因子
                float decay = Mathf.Max(0f, 1f / envDist - 1f / influenceRadius);

                F += tangent * decay * tangentWeight + normal * decay * normalWeight;
            }

            // ---- 4. 合力为零时随机逃逸 ----
            if (F.sqrMagnitude < 1e-6f)
            {
                F = Random.onUnitSphere;
                F.y = Mathf.Abs(F.y);
            }

            Vector3 moveDir = F.normalized;
            Vector3 nextPos = pos + stepSize * moveDir;

            // ---- 5. 姿态插值 ----
            float s = 1f - Mathf.Clamp01(Vector3.Distance(nextPos, goal) / totalStartDist);
            Quaternion nextRot = Quaternion.Slerp(start.rotation, end.rotation, s);
            Pose next = new Pose(nextPos, nextRot);

            // ---- 6. 状态有效性检验：IK + 影子机械臂碰撞 ----
            bool isValid = IsStateValid(next, current, robot, shadowCollisionMonitor, shadowRobotBinder);

            if (isValid)
            {
                consecutiveSkips = 0;
                points.Add(new TcpPathPoint(next, TcpPathPoint.PointType.Approach,
                    TcpPathPoint.PointFlag.Intermediate, seam, robot.Config.TCPMaxSpeed));

                current = next;

                // 收敛判断
                float newDistToGoal = Vector3.Distance(nextPos, goal);
                if (newDistToGoal < tolerance)
                {
                    points.Add(new TcpPathPoint(end, TcpPathPoint.PointType.Approach,
                        TcpPathPoint.PointFlag.End, seam, robot.Config.TCPMaxSpeed));
                    return points;
                }

                // 轨迹震荡检测
                trajectory.Add(nextPos);
                if (trajectory.Count > trajectoryWindow) trajectory.RemoveAt(0);

                if (trajectory.Count >= trajectoryWindow)
                {
                    Vector3 center = Vector3.zero;
                    foreach (var p in trajectory) center += p;
                    center /= trajectory.Count;

                    float distToCenter = Vector3.Distance(nextPos, center);
                    float windowRadius = 0f;
                    foreach (var p in trajectory)
                        windowRadius = Mathf.Max(windowRadius, Vector3.Distance(p, center));

                    if (distToCenter < windowRadius * 0.3f)
                    {
                        // 随机扰动逃逸，解除贴墙锁定
                        Vector3 perturb = Random.onUnitSphere * 0.015f;
                        perturb.y = Mathf.Abs(perturb.y);
                        nextPos = pos + perturb;
                        current = new Pose(nextPos, nextRot);
                        trajectory.Clear();
                        wallFollowLocked = false;
                    }
                }
            }
            else
            {
                consecutiveSkips++;
                if (consecutiveSkips >= maxConsecutiveSkips)
                {
                    Debug.LogWarning($"[APF] Stuck at iteration {i} ({consecutiveSkips} skips), failed.");
                    return null;
                }
            }
        }

        Debug.LogWarning($"[APF] Max iterations ({maxIterations}) reached, failed.");
        return null;
    }

    /// <summary>
    /// 检测两点之间是否可以直接走过去（沿途障碍距离 > 安全裕度）
    /// </summary>
    private static bool CanGoDirect(Vector3 from, Vector3 to, CollisionMonitor monitor)
    {
        const int numSamples = 5;
        const float safeMargin = 0.03f;

        for (int i = 1; i <= numSamples; i++)
        {
            float t = i / (float)numSamples;
            Vector3 sample = Vector3.Lerp(from, to, t);
            Vector3 sampleU = MathUtil.D2UPosition(sample);
            float d = monitor.DistancePointToEnvironment(sampleU, out _);
            if (d < safeMargin) return false;
        }
        return true;
    }

    /// <summary>
    /// 检验目标姿态是否有效：IK 可解 + 影子机械臂无碰撞
    /// </summary>
    private static bool IsStateValid(
        Pose pose, Pose prev,
        RobotModel robot,
        CollisionMonitor shadowMonitor,
        RobotBinder shadowRobotBinder)
    {
        RobotModel shadowRobot = shadowRobotBinder.Robot;

        // IK 求解
        float[] currentJoints = shadowRobot.JointAngles;
        float[] solved = robot.IK.Solve(pose, currentJoints);
        if (solved == null || solved.Length != shadowRobot.JointsCount) return false;

        bool allZero = true;
        foreach (var j in solved) { if (Mathf.Abs(j) > 1e-6f) { allZero = false; break; } }
        if (allZero) return false;

        // 设置影子机械臂关节角
        shadowRobot.SetJointAngles(solved);

        // 运动学计算
        FK.Compute(shadowRobot);

        // 应用 Transform
        shadowRobotBinder.Apply();

        // 同步碰撞体
        Physics.SyncTransforms();

        // 碰撞检测
        shadowMonitor.Update();

        return !shadowMonitor.HasCollision;
    }
}
