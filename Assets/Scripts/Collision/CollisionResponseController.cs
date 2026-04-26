using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 碰撞响应控制器
/// 职责：碰撞等级调度（含预测性触发）
/// 
/// 不包含：
/// - 重规划执行（保留在 SimulationContext.TriggerReplan）
/// - 回滚执行（保留在 SimulationStateMachine / SimulationContext）
/// </summary>
public class CollisionResponseController
{
    // ========== 配置 ==========
    public int PredictionFrames = 2;
    public const float ReplanCooldownDuration = 0.5f;
    public const int MaxConsecutiveWarnings = 50;

    // ========== 状态 ==========
    public float ReplanCooldown { get; private set; } = 0f;
    public int ConsecutiveWarnings { get; private set; } = 0;
    public bool NarrowSpaceMode { get; private set; } = false;
    public bool IsReplanning { get; private set; } = false;

    // ========== 预测结果 ==========
    public class PredictiveResult
    {
        public CollisionMonitor.CollisionLevel CurrentLevel;
        public CollisionMonitor.CollisionLevel PredictedLevel;
        public float PredictionTime;
        public bool ShouldTriggerEarlyReplan;
    }

    // ========== 主入口 ==========

    /// <summary>
    /// 每帧更新，返回是否需要触发重规划（及原因）
    /// </summary>
    public (bool shouldReplan, string reason) Update(
        SimulationContext ctx,
        CollisionMonitor.CollisionLevel currentLevel,
        bool isWorkState,
        bool isTeleop)
    {
        if (ReplanCooldown > 0f)
            ReplanCooldown -= Time.deltaTime;

        var prediction = PredictCollision(ctx);
        return Evaluate(currentLevel, prediction, isWorkState, isTeleop);
    }

    // ========== 预测检测（影子机械臂完整流程）==========

    private PredictiveResult PredictCollision(SimulationContext ctx)
    {
        var result = new PredictiveResult
        {
            CurrentLevel = ctx.CollisionMonitor.OverallLevel,
            PredictedLevel = CollisionMonitor.CollisionLevel.Safe,
            PredictionTime = ctx.Clock.FixedDeltaTime * PredictionFrames,
        };

        float speed = ctx.RobotModel.SmoothedTcpVelocity.magnitude;
        if (speed < 1e-4f) return result;

        // 预测位姿
        Vector3 predictPos = ctx.RobotModel.TCPPosition
            + ctx.RobotModel.SmoothedTcpVelocity * result.PredictionTime;
        Quaternion predictRot = ctx.RobotModel.TCPRotation;
        Pose predictedPose = new Pose(predictPos, predictRot);

        // 影子机械臂 IK
        float[] currentJoints = ctx.RobotModel.JointAngles;
        float[] solved = ctx.RobotModel.IK.Solve(predictedPose, currentJoints);
        if (solved == null || solved.Length != ctx.ShadowRobotModel.JointsCount)
        {
            result.PredictedLevel = CollisionMonitor.CollisionLevel.Warning;
            result.ShouldTriggerEarlyReplan = true;
            return result;
        }

        // 设置影子机械臂 → FK → Apply → 碰撞检测
        ctx.ShadowRobotModel.SetJointAngles(solved);
        FK.Compute(ctx.ShadowRobotModel);
        ctx.ShadowRobotBinder.Apply();
        Physics.SyncTransforms();
        ctx.ShadowCollisionMonitor.Update();

        result.PredictedLevel = ctx.ShadowCollisionMonitor.OverallLevel;
        result.ShouldTriggerEarlyReplan =
            result.CurrentLevel == CollisionMonitor.CollisionLevel.Safe &&
            result.PredictedLevel >= CollisionMonitor.CollisionLevel.Warning;

        return result;
    }

    // ========== 决策逻辑 ==========

    private (bool, string) Evaluate(
        CollisionMonitor.CollisionLevel currentLevel,
        PredictiveResult prediction,
        bool isWorkState,
        bool isTeleop)
    {
        switch (currentLevel)
        {
            case CollisionMonitor.CollisionLevel.Safe:
                return EvaluateSafe(prediction, isWorkState);
            case CollisionMonitor.CollisionLevel.Warning:
                return EvaluateWarning(isWorkState);
            case CollisionMonitor.CollisionLevel.Collision:
                return (false, "Collision");
            default:
                return (false, null);
        }
    }

    private (bool, string) EvaluateSafe(PredictiveResult prediction, bool isWorkState)
    {
        ConsecutiveWarnings = 0;

        if (prediction.ShouldTriggerEarlyReplan && isWorkState && CanReplan())
        {
            return (true, "PredictiveWarning");
        }

        NarrowSpaceMode = false;
        return (false, null);
    }

    private (bool, string) EvaluateWarning(bool isWorkState)
    {
        if (ReplanCooldown > 0f)
        {
            ConsecutiveWarnings++;
            if (!NarrowSpaceMode && ConsecutiveWarnings >= MaxConsecutiveWarnings)
            {
                NarrowSpaceMode = true;
                Debug.Log("[Replan] Narrow space detected.");
            }
            return (false, null);
        }

        if (NarrowSpaceMode)
            return (false, null);

        if (isWorkState && CanReplan())
        {
            return (true, "Warning");
        }

        return (false, null);
    }

    private bool CanReplan()
    {
        return !IsReplanning && ReplanCooldown <= 0f;
    }

    // ========== 外部调用 ==========

    public void OnReplanStarted()
    {
        IsReplanning = true;
        ReplanCooldown = ReplanCooldownDuration;
        ConsecutiveWarnings = 0;
    }

    public void OnReplanFinished()
    {
        IsReplanning = false;
    }

    public void Reset()
    {
        ReplanCooldown = 0f;
        ConsecutiveWarnings = 0;
        NarrowSpaceMode = false;
        IsReplanning = false;
    }
}
