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
