using UnityEngine;

/// <summary>
/// 成功状态：焊接任务成功完成
/// </summary>
public class SucceedState : SimulationStateBase
{
    public SucceedState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        ctx.Clock.Stop();
        ctx.ResultWriter.SetPlanStatus(ctx.TaskState, ctx.Clock.Time);
        ctx.ResultWriter.SaveToJson();

        // 自动运行：尝试开始下一次循环
        if (ctx.TryAutoRun()) return;
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
