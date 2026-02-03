using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Profiling;
using static UnityEngine.GridBrushBase;

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
        // 进入 Working 时可以做准备工作（如果需要）
        ctx.Clock.Start();
        // 规划指令序列
        ctx.WeldPlanner.PlanInstruction(ctx.RobotModel, ctx.Sampler, ctx.WeldTask);
    }

    public override void Update(SimulationContext ctx, float dt)
    {
        // Working 状态负责推进模型仿真步进
        if (ctx == null) return;
        // 规划（生产）
        if (ctx.Trajectory.UnderHighWaterMark)
        {
            ctx.WeldPlanner.PlanTrajectory(
                ctx.RobotModel,
                ctx.Sampler,
                ctx.Trajectory,
                ctx.Clock.Time
            );
        }

        // 执行（消费）
        float[] joints = ctx.Trajectory.Evaluate(ctx.Clock.Time);
        if (joints != null)
        {
            ctx.RobotModel.SetJointAngles(joints);
        }
    }

    public override void Exit(SimulationContext ctx)
    {
        // 退出 Working 时可以做清理工作（如果需要）
        ctx.Clock.Stop();
        // 清空路径队列
        ctx.Trajectory.Clear();
        // 重置规划器
        ctx.WeldPlanner.Reset();
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
        if (ctx == null) return;
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
        if (ctx == null) return;
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
        if (ctx == null) return;
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
        if (ctx == null) return;
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