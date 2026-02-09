using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
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

class IdleState : SimulationStateBase
{
    public IdleState(SimulationStateMachine m) : base(m) { }
    public override void Enter(SimulationContext ctx)
    {
        // Idle 时无需自动步进
        // 可以在此添加进入 Idle 的逻辑

    }

    public override void Exit(SimulationContext ctx)
    {
        // 退出 Idle 时可以做准备工作（如果需要）
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Space) ctx.TryChangeState(SimulationState.Work);
        if (key == KeyCode.LeftShift || key == KeyCode.RightShift) ctx.TryChangeIKMethod();
    }
}

class WorkState : SimulationStateBase
{
    public WorkState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        // 规划路径
        ctx.TcpPathPlanner.Plan(ctx.Task);
        // 可视化路径
        ctx.TcpPathVisualizer.ShowTcpPathPoints(ctx.TcpPathPlanner, 1e10f);
        ctx.Clock.Start();
    }

    public override void Update(SimulationContext ctx, float dt)
    {
        // 取路径点
        List<TcpPathPoint> points = ctx.TcpPathPlanner.TryGetPoints(ctx.TrajectoryPlanner.SegmentsPerPlan);

        // 规划轨迹
        if (ctx.Trajectory.UnderHighWaterMark)
        {
            List<TrajectoryPlanResult> results = ctx.TrajectoryPlanner.Plan(points, ctx.Clock.Time);

            // 处理规划结果
            ctx.TcpPathPlanner.HandleTrajectoryPlanResults(results);
        }

        // 检查路径规划状态
        switch (ctx.TcpPathPlanner.Status)
        {
            case TcpPathPlanner.PlanStatus.Unfinished:
                // 继续仿真
                break;
            case TcpPathPlanner.PlanStatus.Suceeded:
                // 等待轨迹执行结束
                if (!ctx.Trajectory.HasSegment)
                {
                    ctx.TryChangeState(SimulationState.Succeed);
                }
                break;
            case TcpPathPlanner.PlanStatus.Failed:
                // 仿真失败
                ctx.TryChangeState(SimulationState.Fail);
                break;
        }

        // 执行轨迹
        float[] joints = ctx.Trajectory.Evaluate(ctx.Clock.Time);
        if (joints != null)
        {
            ctx.RobotModel.SetJointAngles(joints);
        }
    }

    public override void Exit(SimulationContext ctx)
    {
        
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Space)
        {
            if (ctx.Clock.IsRunning) ctx.Clock.Stop();
            else ctx.Clock.Start();
        }
    }
}

class SucceedState: SimulationStateBase
{
    public SucceedState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        ctx.Clock.Stop();
    }

    public override void Exit(SimulationContext ctx)
    {
        // 重置
        ctx.TcpPathPlanner.Clear();
        ctx.Trajectory.Clear();
        ctx.Clock.Reset();
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Space) ctx.TryChangeState(SimulationState.Idle);
    }
}

class FailState : SimulationStateBase
{
    public FailState(SimulationStateMachine m) : base(m) { }
    public override void Enter(SimulationContext ctx)
    {
        ctx.Clock.Stop();
    }
    public override void Exit(SimulationContext ctx)
    {
        // 重置
        ctx.TcpPathPlanner.Clear();
        ctx.Trajectory.Clear();
        ctx.Clock.Reset();
    }
    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Space) ctx.TryChangeState(SimulationState.Idle);
    }
}

class JointState : SimulationStateBase
{
    public DataChangeSymbol Symbol { get; private set; } = DataChangeSymbol.Stay;

    public int ControlledJoint { get; private set; } = 0;

    public JointState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        // 进入手动关节控制（可能需要 UI 提示等）
    }

    public override void Update(SimulationContext ctx, float dt)
    {
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
        // 重置旋转方向
        Symbol = DataChangeSymbol.Stay;
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (key == KeyCode.LeftArrow) Symbol = DataChangeSymbol.Decrease;
        else if (key == KeyCode.RightArrow) Symbol = DataChangeSymbol.Increase;
        if (num >= 1 && num <= ctx.RobotConfig.JointsParameters.Length)
        {
            ControlledJoint = num - 1;
        }
    }
}

class TCPState : SimulationStateBase
{
    public DataChangeSymbol Symbol { get; private set; } = DataChangeSymbol.Stay;

    public int ControlledData { get; private set; } = 1;

    public TCPState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        // 进入 TCP 控制（可能需要 UI 提示等）
    }

    public override void Update(SimulationContext ctx, float dt)
    {
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
                    default: break;
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
                    default: break;
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
        {
            ControlledData = num;
        }
        if (key == KeyCode.LeftShift || key == KeyCode.RightShift) ctx.TryChangeIKMethod();
    }
}

public enum DataChangeSymbol
{
    Stay,
    Increase,
    Decrease
}