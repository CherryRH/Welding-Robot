using UnityEngine;

/// <summary>
/// 空闲状态：待命，等待用户启动任务或进入遥操作模式
/// </summary>
public class IdleState : SimulationStateBase
{
    public IdleState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx) { }
    public override void Exit(SimulationContext ctx) { }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (ctx == null) return;
        if (key == KeyCode.Space) ctx.TryChangeState(SimulationState.Work);
        if (key == KeyCode.LeftShift || key == KeyCode.RightShift) ctx.TryChangeIKMethod();
    }
}
