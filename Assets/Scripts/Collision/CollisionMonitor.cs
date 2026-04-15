using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 碰撞距离检测模块
/// 支持 BoxCollider 与 CapsuleCollider 的混合检测
/// 
/// 碰撞组：
///   Body  — 机械臂本体（J0~J6，不含焊枪）vs 环境（工作台+工件）
///   Gun   — 焊枪 vs 环境（焊枪靠近工件是正常工况，阈值更小）
///   Self  — 机械臂末端（J4/J5/J6+焊枪）vs 基座侧（J0/J1/J2）
/// 
/// 等级：Safe / Warning / Collision（去除 Blocked）
/// </summary>
public class CollisionMonitor
{
    // ============================================================
    // 每组独立阈值配置（单位：米）
    // ============================================================

    /// <summary>机械臂本体与环境的碰撞阈值</summary>
    public CollisionThresholds BodyThresholds = new CollisionThresholds(
        warningDistance: 0.05f,
        collisionDistance: 0.005f
    );

    /// <summary>焊枪与环境的碰撞阈值（焊枪靠近工件是正常工况，阈值更小）</summary>
    public CollisionThresholds GunThresholds = new CollisionThresholds(
        warningDistance: 0.01f,
        collisionDistance: 0.001f
    );

    /// <summary>自体碰撞阈值</summary>
    public CollisionThresholds SelfThresholds = new CollisionThresholds(
        warningDistance: 0.05f,
        collisionDistance: 0.005f
    );

    // ============================================================
    // 阈值配置结构
    // ============================================================

    public class CollisionThresholds
    {
        /// <summary>预警阈值：距离 ≤ 此值进入 Warning</summary>
        public float WarningDistance;
        /// <summary>碰撞阈值：距离 ≤ 此值（或穿透）判定为 Collision</summary>
        public float CollisionDistance;

        public CollisionThresholds(float warningDistance, float collisionDistance)
        {
            WarningDistance = warningDistance;
            CollisionDistance = collisionDistance;
        }
    }

    // ============================================================
    // 碰撞等级（简化：去除 Blocked）
    // ============================================================

    public enum CollisionLevel
    {
        Safe      = 0,
        Warning   = 1,
        Collision = 2
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
    // 汇总数据（三组）
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

    /// <summary>机械臂本体（J0~J6，不含焊枪）vs 环境</summary>
    public CollisionSummary BodyCollision { get; private set; } = new();

    /// <summary>焊枪 vs 环境</summary>
    public CollisionSummary GunCollision { get; private set; } = new();

    /// <summary>自体碰撞（末端 vs 基座侧）</summary>
    public CollisionSummary SelfCollision { get; private set; } = new();

    // ============================================================
    // 内部碰撞对定义
    // ============================================================

    private struct ColliderPair
    {
        public List<ColliderShape> A;
        public List<ColliderShape> B;
        public string Label;
    }

    private readonly List<ColliderPair> bodyPairs = new();
    private readonly List<ColliderPair> gunPairs  = new();
    private readonly List<ColliderPair> selfPairs = new();

    // 环境碰撞体缓存（供 DistancePointToEnvironment 使用）
    private readonly List<ColliderShape> envShapesCache = new();

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
        bodyPairs.Clear();
        gunPairs.Clear();
        selfPairs.Clear();
        BodyCollision.Pairs.Clear();
        GunCollision.Pairs.Clear();
        SelfCollision.Pairs.Clear();
        envShapesCache.Clear();
        initialized = false;

        // ---- 机械臂各节段包装（BoxColliders[0~6] = J0~J6）----
        int armCount = robot.BoxColliders.Count; // 应为 7
        var armShapes = new List<List<ColliderShape>>(armCount);
        for (int i = 0; i < armCount; i++)
        {
            var shapes = new List<ColliderShape>();
            var boxes = robot.BoxColliders[i];
            if (boxes != null)
                foreach (var b in boxes)
                    if (b != null) shapes.Add(new ColliderShape(b));
            armShapes.Add(shapes);
        }

        // ---- 焊枪（CapsuleCollider）----
        var gunShapes = new List<ColliderShape>();
        if (robot.TorchCollider != null)
            gunShapes.Add(new ColliderShape(robot.TorchCollider));

        // ---- 环境碰撞体（工作台 + 工件）----
        var envShapes = new List<ColliderShape>();
        if (workbench.WorkbenchCollider != null)
            envShapes.Add(new ColliderShape(workbench.WorkbenchCollider));
        foreach (var wc in workbench.WorkpieceColliders)
            if (wc != null) envShapes.Add(new ColliderShape(wc));

        // 缓存供 DistancePointToEnvironment 使用
        envShapesCache.AddRange(envShapes);

        // ----------------------------------------------------------------
        // Body 碰撞对：J0~J6（不含焊枪）vs 环境
        // ----------------------------------------------------------------
        if (envShapes.Count > 0)
        {
            for (int i = 0; i < armCount; i++)
            {
                if (armShapes[i].Count == 0) continue;
                string label = $"Body: J{i} vs Env";
                bodyPairs.Add(new ColliderPair { A = armShapes[i], B = envShapes, Label = label });
                BodyCollision.Pairs.Add(new CollisionPairData { Label = label });
            }
        }

        // ----------------------------------------------------------------
        // Gun 碰撞对：焊枪 vs 环境
        // ----------------------------------------------------------------
        if (envShapes.Count > 0 && gunShapes.Count > 0)
        {
            string gunLabel = "Gun: Torch vs Env";
            gunPairs.Add(new ColliderPair { A = gunShapes, B = envShapes, Label = gunLabel });
            GunCollision.Pairs.Add(new CollisionPairData { Label = gunLabel });
        }

        // ----------------------------------------------------------------
        // Self 碰撞对：末端（J4/J5/J6 + 焊枪）vs 基座侧（J0/J1/J2）
        // 相邻关节物理上不可能碰撞，不检测
        // ----------------------------------------------------------------
        int[] distalIdx   = { 4, 5, 6 };
        int[] proximalIdx = { 0, 1, 2 };

        foreach (int d in distalIdx)
        {
            foreach (int p in proximalIdx)
            {
                if (armShapes[d].Count == 0 || armShapes[p].Count == 0) continue;
                string label = $"Self: J{d} vs J{p}";
                selfPairs.Add(new ColliderPair { A = armShapes[d], B = armShapes[p], Label = label });
                SelfCollision.Pairs.Add(new CollisionPairData { Label = label });
            }
        }

        // 焊枪 vs J0/J1/J2
        if (gunShapes.Count > 0)
        {
            foreach (int p in proximalIdx)
            {
                if (armShapes[p].Count == 0) continue;
                string label = $"Self: Torch vs J{p}";
                selfPairs.Add(new ColliderPair { A = gunShapes, B = armShapes[p], Label = label });
                SelfCollision.Pairs.Add(new CollisionPairData { Label = label });
            }
        }

        initialized = true;
    }

    // ============================================================
    // 每帧更新（常驻调用）
    // ============================================================

    public void Update()
    {
        if (!initialized) return;
        UpdateSummary(bodyPairs, BodyCollision, BodyThresholds);
        UpdateSummary(gunPairs,  GunCollision,  GunThresholds);
        UpdateSummary(selfPairs, SelfCollision, SelfThresholds);
    }

    private void UpdateSummary(List<ColliderPair> pairs, CollisionSummary summary, CollisionThresholds thresholds)
    {
        summary.Reset();

        for (int i = 0; i < pairs.Count; i++)
        {
            var pair = pairs[i];
            var data = summary.Pairs[i];

            float minDist = float.MaxValue;
            Vector3 bestA = Vector3.zero, bestB = Vector3.zero;

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

            data.Distance      = minDist;
            data.ClosestPointA = bestA;
            data.ClosestPointB = bestB;
            data.Level         = ClassifyDistance(minDist, thresholds);

            if (minDist < summary.MinDistance)
            {
                summary.MinDistance = minDist;
                summary.WorstPair   = data;
            }
            if (data.Level > summary.WorstLevel)
                summary.WorstLevel = data.Level;
        }
    }

    // ============================================================
    // 距离计算核心（支持 Box×Box / Box×Capsule / Capsule×Capsule）
    // ============================================================

    private float ComputeDistance(ColliderShape a, ColliderShape b,
                                  out Vector3 ptA, out Vector3 ptB)
    {
        bool penetrating = Physics.ComputePenetration(
            a.Raw, a.Raw.transform.position, a.Raw.transform.rotation,
            b.Raw, b.Raw.transform.position, b.Raw.transform.rotation,
            out _, out float depth
        );

        if (penetrating)
        {
            ptA = a.WorldCenter;
            ptB = b.WorldCenter;
            return -depth;
        }

        Vector3 centerA = a.WorldCenter;
        Vector3 centerB = b.WorldCenter;

        Vector3 onB  = b.ClosestPoint(centerA);
        Vector3 onA  = a.ClosestPoint(onB);
        float d1 = Vector3.Distance(onA, onB);

        Vector3 onA2 = a.ClosestPoint(centerB);
        Vector3 onB2 = b.ClosestPoint(onA2);
        float d2 = Vector3.Distance(onA2, onB2);

        if (d1 <= d2) { ptA = onA;  ptB = onB;  return d1; }
        else          { ptA = onA2; ptB = onB2; return d2; }
    }

    private CollisionLevel ClassifyDistance(float dist, CollisionThresholds t)
    {
        if (dist <= t.CollisionDistance) return CollisionLevel.Collision;
        if (dist <= t.WarningDistance)   return CollisionLevel.Warning;
        return CollisionLevel.Safe;
    }

    // ============================================================
    // 点到环境最近距离查询（供 APF / RRT 等规划算法使用）
    // 只查询环境碰撞体，不依赖机械臂姿态
    // ============================================================

    /// <summary>
    /// 查询空间中某点到所有环境碰撞体（工作台+工件）的最近距离及最近点
    /// 返回值：正数 = 分离距离；负数 = 穿透深度
    /// </summary>
    public float DistancePointToEnvironment(Vector3 worldPos, out Vector3 closestPoint)
    {
        closestPoint = worldPos;
        float minDist = float.MaxValue;

        foreach (var shape in envShapesCache)
        {
            Vector3 onB    = shape.ClosestPoint(worldPos);
            Vector3 onQuery = shape.ClosestPoint(onB);
            float d = Vector3.Distance(worldPos, onQuery);

            if (d < minDist)
            {
                minDist = d;
                closestPoint = onQuery;
            }
        }

        return minDist == float.MaxValue ? BodyThresholds.WarningDistance + 1f : minDist;
    }

    // ============================================================
    // 便捷查询
    // ============================================================

    /// <summary>任意组存在穿透碰撞</summary>
    public bool HasCollision =>
        BodyCollision.WorstLevel == CollisionLevel.Collision ||
        GunCollision.WorstLevel  == CollisionLevel.Collision ||
        SelfCollision.WorstLevel == CollisionLevel.Collision;

    /// <summary>任意组处于 Warning 或更危险状态</summary>
    public bool HasWarning =>
        BodyCollision.WorstLevel >= CollisionLevel.Warning ||
        GunCollision.WorstLevel  >= CollisionLevel.Warning ||
        SelfCollision.WorstLevel >= CollisionLevel.Warning;

    /// <summary>当前整体最危险等级</summary>
    public CollisionLevel OverallLevel =>
        (CollisionLevel)Mathf.Max(
            Mathf.Max((int)BodyCollision.WorstLevel, (int)GunCollision.WorstLevel),
            (int)SelfCollision.WorstLevel
        );

    /// <summary>
    /// 路径规划安全性检查：三组均为 Safe，且各组最近距离 >= 对应 Collision 阈值 + margin
    /// </summary>
    /// <param name="margin">额外安全裕度（米）</param>
    public bool IsSafeForPlanning(float margin = 0.005f)
    {
        if (HasCollision) return false;

        if (BodyCollision.MinDistance < BodyThresholds.CollisionDistance + margin) return false;
        if (GunCollision.MinDistance  < GunThresholds.CollisionDistance  + margin) return false;
        if (SelfCollision.MinDistance < SelfThresholds.CollisionDistance + margin) return false;

        return true;
    }
}
