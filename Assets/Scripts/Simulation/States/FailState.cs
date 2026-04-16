using UnityEngine;

/// <summary>
/// 失败状态：焊接任务失败（碰撞、规划失败等）
/// </summary>
public class FailState : SimulationStateBase
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
        ctx.Reset();
        if (key == KeyCode.Escape || key == KeyCode.Space)
        {
            ctx.TryChangeState(SimulationState.Idle);
        }
        else if (key == KeyCode.Tab)
        {
            ctx.TryChangeState(SimulationState.Replay);
        }
    }
}
