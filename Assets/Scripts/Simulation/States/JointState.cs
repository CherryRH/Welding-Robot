using UnityEngine;

/// <summary>
/// 关节遥操作状态：直接控制单个关节旋转
/// </summary>
public class JointState : SimulationStateBase
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
