using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 碰撞距离检测模块
/// 支持 BoxCollider 与 CapsuleCollider 的混合检测
/// 在 SimulationStateMachine.Update 中作为常驻调用
/// </summary>
public class CollisionMonitor
{
    // ============================================================
    // 阈值配置（单位：米）
    // ============================================================

    /// <summary>预警阈值：距离 ≤ 此值进入 Warning</summary>
    public float SafeDistance = 0.05f;

    /// <summary>阻滞阈值：距离 ≤ 此值进入 Blocked</summary>
    public float BlockedDistance = 0.005f;

    /// <summary>碰撞阈值：距离 ≤ 此值（或穿透）判定为 Collision</summary>
    public float CollisionDistance = 0.001f;

    // ============================================================
    // 碰撞等级
    // ============================================================

    public enum CollisionLevel
    {
        Safe = 0,
        Warning = 1,
        Blocked = 2,
        Collision = 3
    }

    // ============================================================
    // 统一碰撞体包装
    // ============================================================

    /// <summary>
    /// 对 BoxCollider / CapsuleCollider 的统一包装
    /// 后续如需支持更多类型，只需扩展此类
    /// </summary>
    public class ColliderShape
    {
        public readonly Collider Raw;

        public ColliderShape(Collider c)
        {
            Raw = c != null ? c : throw new ArgumentNullException(nameof(c));
        }

        public bool IsValid => Raw != null;

        public Vector3 WorldCenter
        {
            get
            {
                return Raw switch
                {
                    BoxCollider box => box.transform.TransformPoint(box.center),
                    CapsuleCollider cap => cap.transform.TransformPoint(cap.center),
                    _ => Raw.transform.position,
                };
            }
        }

        /// <summary>
        /// 返回 Collider 表面上距 worldPoint 最近的点
        /// </summary>
        public Vector3 ClosestPoint(Vector3 worldPoint)
            => Raw.ClosestPoint(worldPoint);
    }

    // ============================================================
    // 碰撞对数据
    // ============================================================

    public class CollisionPairData
    {
        public string Label;
        /// <summary>当前最近距离（米）；负值表示穿透深度</summary>
        public float Distance = float.MaxValue;
        public CollisionLevel Level = CollisionLevel.Safe;
        /// <summary>A 侧最近点（世界坐标）</summary>
        public Vector3 ClosestPointA;
        /// <summary>B 侧最近点（世界坐标）</summary>
        public Vector3 ClosestPointB;
    }

    // ============================================================
    // 汇总数据（两套）
    // ============================================================

    public class CollisionSummary
    {
        public readonly List<CollisionPairData> Pairs = new();
        public CollisionLevel WorstLevel = CollisionLevel.Safe;
        public float MinDistance = float.MaxValue;
        public CollisionPairData WorstPair;

        public void Reset()
        {
            WorstLevel = CollisionLevel.Safe;
            MinDistance = float.MaxValue;
            WorstPair = null;
        }
    }

    /// <summary>自体碰撞汇总（J4/J5/J6 vs J0/J1/J2 + Torch vs J0/J1/J2）</summary>
    public CollisionSummary SelfCollision { get; private set; } = new();

    /// <summary>环境碰撞汇总（机械臂全节段 + Torch vs 工作台/工件）</summary>
    public CollisionSummary EnvCollision { get; private set; } = new();

    // ============================================================
    // 内部碰撞对定义
    // ============================================================

    private struct ColliderPair
    {
        /// <summary>A 侧碰撞体列表（可混合 Box/Capsule）</summary>
        public List<ColliderShape> A;
        /// <summary>B 侧碰撞体列表（可混合 Box/Capsule）</summary>
        public List<ColliderShape> B;
        public string Label;
    }

    private readonly List<ColliderPair> selfPairs = new();
    private readonly List<ColliderPair> envPairs = new();

    private bool initialized = false;
    public bool IsInitialized => initialized;

    // ============================================================
    // 初始化
    // ============================================================

    /// <summary>
    /// 构建碰撞对列表
    /// 在 SimulationContext.Build() 末尾调用（WorkpieceColliders 已就绪）
    /// </summary>
    public void Init(RobotBinder robot, WorkbenchBinder workbench)
    {
        selfPairs.Clear();
        envPairs.Clear();
        SelfCollision.Pairs.Clear();
        EnvCollision.Pairs.Clear();
        initialized = false;

        // ---- 将机械臂各节段包装为 ColliderShape 列表 ----
        // arm[0~6] = J0(底座)~J6，arm[7] = Torch（CapsuleCollider 单独处理）
        // BoxColliders.Count == 7，索引 0~6 对应 J0~J6
        int armCount = robot.BoxColliders.Count; // 应为 7

        // 构建每节段的 shape 列表（Box）
        var armShapes = new List<List<ColliderShape>>(armCount);
        for (int i = 0; i < armCount; i++)
        {
            var shapes = new List<ColliderShape>();
            var boxes = robot.BoxColliders[i];
            if (boxes != null)
            {
                foreach (var b in boxes)
                    if (b != null) shapes.Add(new ColliderShape(b));
            }
            armShapes.Add(shapes);
        }

        // Torch 单独作为一个节段（CapsuleCollider）
        var torchShapes = new List<ColliderShape>();
        if (robot.TorchCollider != null)
            torchShapes.Add(new ColliderShape(robot.TorchCollider));

        // ----------------------------------------------------------------
        // 自体碰撞对
        //
        // 末端侧（distal）：J4(idx=4), J5(idx=5), J6(idx=6), Torch
        // 基座侧（proximal）：J0(idx=0), J1(idx=1), J2(idx=2)
        //
        // 相邻关节（J3↔J4, J4↔J5 等）物理上不可能碰撞，不检测
        // ----------------------------------------------------------------
        int[] distalArmIdx = { 4, 5, 6 };
        int[] proximalArmIdx = { 0, 1, 2 };

        // J4/J5/J6 vs J0/J1/J2
        foreach (int d in distalArmIdx)
        {
            foreach (int p in proximalArmIdx)
            {
                if (armShapes[d].Count == 0 || armShapes[p].Count == 0) continue;
                string label = $"Self: J{d} vs J{p}";
                selfPairs.Add(new ColliderPair { A = armShapes[d], B = armShapes[p], Label = label });
                SelfCollision.Pairs.Add(new CollisionPairData { Label = label });
            }
        }

        // Torch vs J0/J1/J2
        if (torchShapes.Count > 0)
        {
            foreach (int p in proximalArmIdx)
            {
                if (armShapes[p].Count == 0) continue;
                string label = $"Self: Torch vs J{p}";
                selfPairs.Add(new ColliderPair { A = torchShapes, B = armShapes[p], Label = label });
                SelfCollision.Pairs.Add(new CollisionPairData { Label = label });
            }
        }

        // ----------------------------------------------------------------
        // 环境碰撞对
        //
        // 机械臂侧：J0~J6 + Torch
        // 环境侧：WorkbenchCollider + WorkpieceColliders[]
        // ----------------------------------------------------------------
        var envShapes = new List<ColliderShape>();
        if (workbench.WorkbenchCollider != null)
            envShapes.Add(new ColliderShape(workbench.WorkbenchCollider));
        foreach (var wc in workbench.WorkpieceColliders)
            if (wc != null) envShapes.Add(new ColliderShape(wc));

        if (envShapes.Count == 0)
        {
            initialized = true;
            return;
        }

        // J0~J6
        for (int i = 0; i < armCount; i++)
        {
            if (armShapes[i].Count == 0) continue;
            string label = $"Env: J{i} vs Workbench/Workpiece";
            envPairs.Add(new ColliderPair { A = armShapes[i], B = envShapes, Label = label });
            EnvCollision.Pairs.Add(new CollisionPairData { Label = label });
        }

        // Torch
        if (torchShapes.Count > 0)
        {
            string label = "Env: Torch vs Workbench/Workpiece";
            envPairs.Add(new ColliderPair { A = torchShapes, B = envShapes, Label = label });
            EnvCollision.Pairs.Add(new CollisionPairData { Label = label });
        }

        initialized = true;
    }

    // ============================================================
    // 每帧更新（常驻调用）
    // ============================================================

    public void Update()
    {
        if (!initialized) return;
        UpdateSummary(selfPairs, SelfCollision);
        UpdateSummary(envPairs, EnvCollision);
    }

    private void UpdateSummary(List<ColliderPair> pairs, CollisionSummary summary)
    {
        summary.Reset();

        for (int i = 0; i < pairs.Count; i++)
        {
            var pair = pairs[i];
            var data = summary.Pairs[i];

            float minDist = float.MaxValue;
            Vector3 bestA = Vector3.zero, bestB = Vector3.zero;

            // A 侧 × B 侧，取所有子碰撞箱中的最小距离
            foreach (var shapeA in pair.A)
            {
                foreach (var shapeB in pair.B)
                {
                    float d = ComputeDistance(shapeA, shapeB, out Vector3 ptA, out Vector3 ptB);
                    if (d < minDist)
                    {
                        minDist = d;
                        bestA = ptA;
                        bestB = ptB;
                    }
                }
            }

            data.Distance = minDist;
            data.ClosestPointA = bestA;
            data.ClosestPointB = bestB;
            data.Level = ClassifyDistance(minDist);

            // 更新汇总
            if (minDist < summary.MinDistance)
            {
                summary.MinDistance = minDist;
                summary.WorstPair = data;
            }
            if (data.Level > summary.WorstLevel)
                summary.WorstLevel = data.Level;
        }
    }

    // ============================================================
    // 距离计算核心
    // 支持 Box×Box / Box×Capsule / Capsule×Capsule
    // ============================================================

    /// <summary>
    /// 计算两个 ColliderShape 之间的最近距离
    /// 返回值：正数 = 分离距离（米）；负数 = 穿透深度（米）
    /// </summary>
    private float ComputeDistance(ColliderShape a, ColliderShape b,
                                  out Vector3 ptA, out Vector3 ptB)
    {
        // 1. 先用 ComputePenetration 检测穿透（适用于所有 Collider 组合）
        bool penetrating = Physics.ComputePenetration(
            a.Raw, a.Raw.transform.position, a.Raw.transform.rotation,
            b.Raw, b.Raw.transform.position, b.Raw.transform.rotation,
            out _, out float depth
        );

        if (penetrating)
        {
            ptA = a.WorldCenter;
            ptB = b.WorldCenter;
            return -depth; // 负值表示穿透
        }

        // 2. 未穿透：双向 ClosestPoint 取最小
        //    ClosestPoint 对 Box 和 Capsule 均有效
        Vector3 centerA = a.WorldCenter;
        Vector3 centerB = b.WorldCenter;

        // A 中心 → B 表面最近点，再反查 A 表面
        Vector3 onB = b.ClosestPoint(centerA);
        Vector3 onA = a.ClosestPoint(onB);
        float d1 = Vector3.Distance(onA, onB);

        // B 中心 → A 表面最近点，再反查 B 表面
        Vector3 onA2 = a.ClosestPoint(centerB);
        Vector3 onB2 = b.ClosestPoint(onA2);
        float d2 = Vector3.Distance(onA2, onB2);

        if (d1 <= d2)
        {
            ptA = onA;
            ptB = onB;
            return d1;
        }
        else
        {
            ptA = onA2;
            ptB = onB2;
            return d2;
        }
    }

    private CollisionLevel ClassifyDistance(float dist)
    {
        if (dist <= CollisionDistance) return CollisionLevel.Collision;
        if (dist <= BlockedDistance) return CollisionLevel.Blocked;
        if (dist <= SafeDistance) return CollisionLevel.Warning;
        return CollisionLevel.Safe;
    }

    // ============================================================
    // 点到环境最近距离查询（供 APF 等规划算法使用）
    // ============================================================

    /// <summary>
    /// 查询空间中某点到所有环境碰撞体（工作台+工件）的最近距离及最近点
    /// 返回值：正数 = 分离距离；0 = 恰好接触；负数 = 穿透深度
    /// </summary>
    /// <param name="worldPos">世界坐标系中的查询点</param>
    /// <param name="closestPoint">最近点位置（世界坐标）</param>
    /// <returns>最近距离（米）</returns>
    public float DistancePointToEnvironment(Vector3 worldPos, out Vector3 closestPoint)
    {
        closestPoint = worldPos;
        float minDist = float.MaxValue;

        foreach (var envPair in envPairs)
        {
            foreach (var shapeB in envPair.B)
            {
                // 用双向 ClosestPoint 估算点到碰撞体的最近距离
                Vector3 onB = shapeB.ClosestPoint(worldPos);
                Vector3 onQuery = shapeB.ClosestPoint(onB);
                float d = Vector3.Distance(worldPos, onQuery);

                if (d < minDist)
                {
                    minDist = d;
                    closestPoint = onQuery;
                }
            }
        }

        // 如果没有环境碰撞对，返回安全距离
        if (minDist == float.MaxValue)
            return SafeDistance + 1f;

        return minDist;
    }

    // ============================================================
    // 便捷查询
    // ============================================================

    /// <summary>当前是否存在任何碰撞（穿透）</summary>
    public bool HasCollision =>
        SelfCollision.WorstLevel == CollisionLevel.Collision ||
        EnvCollision.WorstLevel == CollisionLevel.Collision;

    /// <summary>当前是否处于阻滞或更危险状态</summary>
    public bool IsBlocked =>
        SelfCollision.WorstLevel >= CollisionLevel.Blocked ||
        EnvCollision.WorstLevel >= CollisionLevel.Blocked;

    /// <summary>当前整体最危险等级</summary>
    public CollisionLevel OverallLevel =>
        (CollisionLevel)Mathf.Max(
            (int)SelfCollision.WorstLevel,
            (int)EnvCollision.WorstLevel
        );
}
