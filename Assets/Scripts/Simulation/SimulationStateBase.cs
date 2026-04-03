using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 仿真状态基类
/// </summary>
public abstract class SimulationStateBase
{
    protected readonly SimulationStateMachine Machine;
    protected SimulationStateBase(SimulationStateMachine machine) => Machine = machine;

    public virtual void Enter(SimulationContext ctx) { }
    public virtual void Exit(SimulationContext ctx) { }
    public virtual void Update(SimulationContext ctx, float dt) { }
    public virtual void HandleInput(SimulationContext ctx, KeyCode key, int num) { }
}

/// <summary>
/// 空闲状态：待命，等待用户启动任务或进入遥操作模式
/// </summary>
class IdleState : SimulationStateBase
{
    public IdleState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        
    }

    public override void Exit(SimulationContext ctx) { }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Space) ctx.TryChangeState(SimulationState.Work);
        if (key == KeyCode.LeftShift || key == KeyCode.RightShift) ctx.TryChangeIKMethod();
    }
}

/// <summary>
/// 工作状态：执行焊接任务
/// </summary>
class WorkState : SimulationStateBase
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

        // 启动仿真时钟
        ctx.Clock.Start();
    }

    public override void Update(SimulationContext ctx, float dt)
    {
        // 持续规划轨迹（保持轨迹缓冲区水位）
        if (ctx.Trajectory.UnderHighWaterMark && ctx.TaskState.Status != WeldTaskPlanState.PlanStatus.Failed)
        {
            List<TcpPathPoint> points = ctx.TcpPathPlanner.GetPathPart();
            TrajectoryPlanResult result = ctx.TrajectoryPlanner.Plan(points, ctx.Clock.Time);
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

        // 焊接特效
        if (ctx.Trajectory.CurrentSegment != null && ctx.Trajectory.CurrentSegment.Type == TrajectorySegment.TrajectorySegmentType.Weld)
            ctx.EffectBinder.PlayWeldingEffect();
        else
            ctx.EffectBinder.StopWeldingEffect();
    }

    public override void Exit(SimulationContext ctx)
    {
        // 停止焊接特效
        ctx.EffectBinder.StopWeldingEffect();
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

/// <summary>
/// 成功状态：焊接任务成功完成
/// </summary>
class SucceedState : SimulationStateBase
{
    public SucceedState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        ctx.Clock.Stop();
        ctx.ResultWriter.SetPlanStatus(WeldTaskPlanState.PlanStatus.Suceeded);
        ctx.ResultWriter.SaveToJson();
    }

    public override void Exit(SimulationContext ctx)
    {
        // 清空规划数据
        ctx.Clear();
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Escape || key == KeyCode.Space)
        {
            ctx.TryChangeState(SimulationState.Idle);
            ctx.Reset();
        }
    }
}

/// <summary>
/// 失败状态：焊接任务失败（碰撞、规划失败等）
/// </summary>
class FailState : SimulationStateBase
{
    public FailState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        ctx.Clock.Stop();
        ctx.ResultWriter.SetPlanStatus(WeldTaskPlanState.PlanStatus.Failed);
        ctx.ResultWriter.SaveToJson();
    }

    public override void Exit(SimulationContext ctx)
    {
        // 清空规划数据
        ctx.Clear();
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Escape || key == KeyCode.Space)
        {
            ctx.TryChangeState(SimulationState.Idle);
            ctx.Reset();
        }
    }
}

/// <summary>
/// 关节遥操作状态：直接控制单个关节旋转
/// </summary>
class JointState : SimulationStateBase
{
    public DataChangeSymbol Symbol { get; private set; } = DataChangeSymbol.Stay;
    public int ControlledJoint { get; private set; } = 0;

    public JointState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx) { }

    public override void Update(SimulationContext ctx, float dt)
    {
        // 碰撞检测由 SimulationContext 统一处理
        switch (Symbol)
        {
            case DataChangeSymbol.Increase:
                ctx.RobotModel.SingleJointRotationStep(dt, ControlledJoint, true);
                break;
            case DataChangeSymbol.Decrease:
                ctx.RobotModel.SingleJointRotationStep(dt, ControlledJoint, false);
                break;
            case DataChangeSymbol.Stay:
            default:
                break;
        }
        Symbol = DataChangeSymbol.Stay;
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (key == KeyCode.LeftArrow) Symbol = DataChangeSymbol.Decrease;
        else if (key == KeyCode.RightArrow) Symbol = DataChangeSymbol.Increase;
        if (num >= 1 && num <= ctx.RobotConfig.JointsParameters.Length)
            ControlledJoint = num - 1;
    }
}

/// <summary>
/// TCP 遥操作状态：直接控制 TCP 平移/旋转
/// </summary>
class TCPState : SimulationStateBase
{
    public DataChangeSymbol Symbol { get; private set; } = DataChangeSymbol.Stay;
    public int ControlledData { get; private set; } = 1;

    public TCPState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx) { }

    public override void Update(SimulationContext ctx, float dt)
    {
        // 碰撞检测由 SimulationContext 统一处理
        switch (Symbol)
        {
            case DataChangeSymbol.Increase:
                switch (ControlledData)
                {
                    case 1: ctx.RobotModel.TCPMoveStep(dt, new(1f, 0, 0), new(0, 0, 0)); break;
                    case 2: ctx.RobotModel.TCPMoveStep(dt, new(0, 1f, 0), new(0, 0, 0)); break;
                    case 3: ctx.RobotModel.TCPMoveStep(dt, new(0, 0, 1f), new(0, 0, 0)); break;
                    case 4: ctx.RobotModel.TCPMoveStep(dt, new(0, 0, 0), new(1f, 0, 0)); break;
                    case 5: ctx.RobotModel.TCPMoveStep(dt, new(0, 0, 0), new(0, 1f, 0)); break;
                    case 6: ctx.RobotModel.TCPMoveStep(dt, new(0, 0, 0), new(0, 0, 1f)); break;
                }
                break;
            case DataChangeSymbol.Decrease:
                switch (ControlledData)
                {
                    case 1: ctx.RobotModel.TCPMoveStep(dt, new(-1f, 0, 0), new(0, 0, 0)); break;
                    case 2: ctx.RobotModel.TCPMoveStep(dt, new(0, -1f, 0), new(0, 0, 0)); break;
                    case 3: ctx.RobotModel.TCPMoveStep(dt, new(0, 0, -1f), new(0, 0, 0)); break;
                    case 4: ctx.RobotModel.TCPMoveStep(dt, new(0, 0, 0), new(-1f, 0, 0)); break;
                    case 5: ctx.RobotModel.TCPMoveStep(dt, new(0, 0, 0), new(0, -1f, 0)); break;
                    case 6: ctx.RobotModel.TCPMoveStep(dt, new(0, 0, 0), new(0, 0, -1f)); break;
                }
                break;
            case DataChangeSymbol.Stay:
            default:
                break;
        }
        Symbol = DataChangeSymbol.Stay;
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (key == KeyCode.LeftArrow) Symbol = DataChangeSymbol.Decrease;
        else if (key == KeyCode.RightArrow) Symbol = DataChangeSymbol.Increase;
        if (num >= 1 && num <= 6)
            ControlledData = num;
        if (key == KeyCode.LeftShift || key == KeyCode.RightShift) ctx.TryChangeIKMethod();
    }
}

/// <summary>
/// 移动方向符号
/// </summary>
public enum DataChangeSymbol
{
    Stay,
    Increase,
    Decrease
}