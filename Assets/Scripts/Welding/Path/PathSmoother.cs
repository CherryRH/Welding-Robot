using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 路径平滑器：对已规划的路径进行后处理优化
/// 
/// 基于 APF（人工势场）思想，通过四种力进行迭代微调：
/// 1. 引力（Attraction）：目标点产生的吸引力，引导路径向终点收缩
/// 2. 拉直力（Pull to Line）：将中间节点向起点-终点连线拉拢，减少绕路
/// 3. 平滑力（Laplacian）：将节点向相邻节点中点移动，消除抖动
/// 4. 排斥力（Repulsion）：障碍物产生的排斥力，保持安全距离
/// 
/// 每次移动后进行 IK + 碰撞检验，保证路径有效性
/// </summary>
public static class PathSmoother
{
    /// <summary>
    /// 平滑配置参数
    /// </summary>
    public class Config
    {
        /// <summary>迭代轮数，建议 10-30</summary>
        public int Iterations = 10;

        /// <summary>目标点引力强度，建议 0.02-0.1</summary>
        public float AttractStrength = 0.05f;

        /// <summary>拉直力强度，范围 [0, 1]，0=不拉直</summary>
        public float PullStrength = 0.1f;

        /// <summary>平滑力强度，范围 [0, 1]，0=不平滑</summary>
        public float SmoothStrength = 0.1f;

        /// <summary>排斥力强度，建议 0.01-0.1</summary>
        public float RepulsionStrength = 0.1f;

        /// <summary>排斥力作用半径（米）</summary>
        public float RepulsionRadius = 0.05f;

        /// <summary>碰撞检验安全裕度（米）</summary>
        public float Margin = 0.01f;
    }

    /// <summary>
    /// 对路径进行平滑优化
    /// </summary>
    /// <param name="path">原始路径（起点和终点固定不动）</param>
    /// <param name="robot">机械臂模型</param>
    /// <param name="shadowCollisionMonitor">影子碰撞监测器</param>
    /// <param name="shadowRobotBinder">影子机械臂绑定器</param>
    /// <param name="config">平滑配置</param>
    /// <returns>平滑后的路径</returns>
    public static List<TcpPathPoint> Smooth(
        List<TcpPathPoint> path,
        RobotModel robot,
        CollisionMonitor shadowCollisionMonitor,
        RobotBinder shadowRobotBinder,
        Config config = null)
    {
        if (path == null || path.Count <= 2)
            return path;

        config ??= new Config();

        Debug.Log($"[Smoother] Starting: {path.Count} nodes, " +
            $"iterations={config.Iterations}, attract={config.AttractStrength:F3}, " +
            $"pull={config.PullStrength:F2}, smooth={config.SmoothStrength:F2}, " +
            $"repulsion={config.RepulsionStrength:F3}");

        // 复制路径，避免修改原始数据
        var points = new List<TcpPathPoint>(path);
        Pose start = points[0].Pose;
        Pose end = points[^1].Pose;

        // 计算垂直平面法向量（在 xOy 平面内），用于拉直力
        // 该平面经过起点和终点，只约束水平方向，高度（Z）自由
        // 这样翻越障碍物的向上弧线不会被压低
        Vector3 delta = end.position - start.position;
        Vector3 planeNormal = new Vector3(delta.x, delta.y, 0f); // Data 坐标系 up=+Z
        if (planeNormal.sqrMagnitude < 1e-10f)
            planeNormal = Vector3.right; // 起点/终点垂直对齐时退化为 X 方向
        else
            planeNormal.Normalize();

        float totalDist = Vector3.Distance(start.position, end.position);

        // 避免除零
        if (totalDist < 1e-6f) return points;

        int improvedCount = 0;

        for (int iter = 0; iter < config.Iterations; iter++)
        {
            int iterImproved = 0;

            // 遍历中间节点（起点和终点不动）
            for (int i = 1; i < points.Count - 1; i++)
            {
                TcpPathPoint prev = points[i - 1];
                TcpPathPoint curr = points[i];
                TcpPathPoint next = points[i + 1];

                Vector3 pos = curr.Pose.position;
                Vector3 newPos = pos;

                // ========== 1. 引力（目标点吸引力）==========
                // APF 引力场：距离越远引力越强
                // F = k * d / totalDist（归一化，避免过大）
                Vector3 toEnd = end.position - pos;
                float distToEnd = toEnd.magnitude;
                if (distToEnd > 1e-6f)
                {
                    Vector3 attractDir = toEnd.normalized;
                    float attractMag = config.AttractStrength * (distToEnd / totalDist);
                    newPos += attractDir * attractMag;
                }

                // ========== 2. 拉直力（向垂直平面靠拢）==========
                // 将节点向经过起点-终点的垂直平面投影
                // 只约束水平方向（xOy），高度（Z）不受影响
                // 允许路径自由翻越障碍物，同时减少水平方向绕路
                float distToPlane = Vector3.Dot(pos - start.position, planeNormal);
                Vector3 projection = pos - distToPlane * planeNormal;
                newPos = Vector3.Lerp(newPos, projection, config.PullStrength);

                // ========== 3. 平滑力 ==========
                // 拉普拉斯平滑：向相邻节点中点移动
                Vector3 midpoint = (prev.Pose.position + next.Pose.position) * 0.5f;
                newPos = Vector3.Lerp(newPos, midpoint, config.SmoothStrength);

                // ========== 4. 排斥力 ==========
                // APF 斥力场：距离越近排斥越强
                Vector3 posU = MathUtil.D2UPosition(newPos);
                float envDist = shadowCollisionMonitor.DistancePointToEnvironment(posU, out Vector3 closestU);
                Vector3 closest = MathUtil.U2DPosition(closestU);

                if (envDist < config.RepulsionRadius && envDist > 1e-6f)
                {
                    Vector3 repulsionDir = (newPos - closest).normalized;
                    float repulsionMag = config.RepulsionStrength * (1f - envDist / config.RepulsionRadius);
                    newPos += repulsionDir * repulsionMag;
                }

                // ========== 5. 姿态插值 ==========
                // 姿态不参与优化，始终按距离比例插值
                float s = Mathf.Clamp01(Vector3.Distance(newPos, start.position) / totalDist);
                Quaternion newRot = Quaternion.Slerp(start.rotation, end.rotation, s);
                Pose newPose = new Pose(newPos, newRot);

                // ========== 6. 有效性检验 ==========
                if (IsStateValid(newPose, prev.Pose, robot, shadowCollisionMonitor, shadowRobotBinder, config.Margin))
                {
                    points[i] = new TcpPathPoint(
                        newPose,
                        curr.Type,
                        curr.Flag,
                        curr.Seam,
                        curr.Speed);
                    iterImproved++;
                }
                // 检验失败则保持原位
            }

            improvedCount += iterImproved;

            // 本轮没有任何改进，提前终止
            if (iterImproved == 0 && iter > 2)
            {
                Debug.Log($"[Smoother] Early stop at iter {iter + 1}, no improvement");
                break;
            }
        }

        Debug.Log($"[Smoother] Done: {improvedCount} position updates across {config.Iterations} iterations");

        return points;
    }

    /// <summary>
    /// 检验目标姿态是否有效：IK 可解 + 无碰撞
    /// </summary>
    private static bool IsStateValid(
        Pose pose,
        Pose prevPose,
        RobotModel robot,
        CollisionMonitor shadowMonitor,
        RobotBinder shadowRobotBinder,
        float margin)
    {
        RobotModel shadowRobot = shadowRobotBinder.Robot;

        // IK 求解
        float[] currentJoints = shadowRobot.JointAngles;
        float[] solved = robot.IK.Solve(pose, currentJoints);

        if (solved == null || solved.Length != shadowRobot.JointsCount)
            return false;

        // 检查是否全零（IK 失败）
        bool allZero = true;
        foreach (var j in solved)
        {
            if (Mathf.Abs(j) > 1e-6f)
            {
                allZero = false;
                break;
            }
        }
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

        // 检查是否安全
        return shadowMonitor.IsSafeForPlanning(margin);
    }
}
