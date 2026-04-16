using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 工作状态：执行焊接任务
/// </summary>
public class WorkState : SimulationStateBase
{
    public WorkState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        // 规划 TCP 路径
        ctx.TcpPathPlanner.Plan(ctx.Task, ctx.ShadowCollisionMonitor, ctx.ShadowRobotBinder);

        // 显示路径可视化
        ctx.TcpPathVisualizer.ShowTcpPathPoints(ctx.TcpPathPlanner);

        // 初始化结果记录器
        ctx.ResultWriter.Init(ctx.Task?.TaskName ?? "UnknownTask");

        // 重置实时数据状态（速度、加速度等）
        ctx.RobotModel.ResetRealtimeData();

        // 启动仿真时钟
        ctx.Clock.Start();
    }

    public override void Update(SimulationContext ctx, float dt)
    {
        // 持续规划轨迹（保持轨迹缓冲区水位）
        if (ctx.Trajectory.UnderHighWaterMark && ctx.TaskState.Status != WeldTaskPlanState.PlanStatus.Failed)
        {
            // 获取 TCP 路径点并规划轨迹
            List<TcpPathPoint> points = ctx.TcpPathPlanner.GetPathPart();
            TrajectoryPlanResult result = ctx.TrajectoryPlanner.Plan(points, ctx.Clock.Time);

            // 处理规划结果
            ctx.TcpPathPlanner.HandleTrajectoryPlanResult(result);

            // 规划状态变化时更新可视化
            if (result.PlanStatus != TrajectoryPlanResult.TrajectoryPlanStatus.Ok)
                ctx.TcpPathVisualizer.ShowTcpPathPoints(ctx.TcpPathPlanner);
        }

        // 检查任务规划状态
        switch (ctx.TaskState.Status)
        {
            case WeldTaskPlanState.PlanStatus.Unfinished:
                break;
            case WeldTaskPlanState.PlanStatus.Suceeded:
                if (!ctx.Trajectory.HasActiveSegment)
                    ctx.TryChangeState(SimulationState.Succeed);
                break;
            case WeldTaskPlanState.PlanStatus.Failed:
                if (!ctx.Trajectory.HasActiveSegment)
                    ctx.TryChangeState(SimulationState.Fail);
                break;
        }

        // 执行轨迹
        float[] joints = ctx.Trajectory.Evaluate(ctx.Clock.Time, out TrajectorySegment finishedSegment);
        if (joints != null)
            ctx.RobotModel.SetJointAngles(joints);

        // 记录完成的焊接路径段
        if (finishedSegment != null)
            ctx.ResultWriter.RecordSegment(finishedSegment);

        // 焊接特效 + 熔池可视化
        if (ctx.Trajectory.CurrentSegment != null && ctx.Trajectory.CurrentSegment.Type == WeldStateType.Weld)
        {
            ctx.EffectBinder.PlayWeldingEffect();
            // 焊接段开始时创建新条带
            if (ctx.MoltenPoolVisualizer != null && !ctx.MoltenPoolVisualizer.IsStripActive)
                ctx.MoltenPoolVisualizer.StartStrip();
        }
        else
        {
            ctx.EffectBinder.StopWeldingEffect();
            // 焊接段结束时冻结当前条带
            if (ctx.MoltenPoolVisualizer != null && ctx.MoltenPoolVisualizer.IsStripActive)
                ctx.MoltenPoolVisualizer.FinalizeStrip();
        }
    }

    public override void Exit(SimulationContext ctx)
    {
        // 停止焊接特效
        ctx.EffectBinder.StopWeldingEffect();
        // 冻结当前熔池条带
        if (ctx.MoltenPoolVisualizer != null && ctx.MoltenPoolVisualizer.IsStripActive)
            ctx.MoltenPoolVisualizer.FinalizeStrip();
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Space)
        {
            // 暂停/继续
            if (ctx.Clock.IsRunning) ctx.Clock.Stop();
            else ctx.Clock.Start();
        }
        if (key == KeyCode.Escape)
        {
            // 强制退出到 Idle
            ctx.Clock.Stop();
            ctx.TryChangeState(SimulationState.Idle);
        }
    }
}
