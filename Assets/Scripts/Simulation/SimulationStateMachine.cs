using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 仿真状态机
/// </summary>
public class SimulationStateMachine
{
    public SimulationState CurrentState { get; private set; } = SimulationState.Idle;

    public int FocusedVariable { get; private set; } = 0;

    public event Action<SimulationState, SimulationState> OnStateChange;

    private readonly Dictionary<SimulationState, SimulationStateBase> states;

    // ========== 碰撞回滚支持 ==========

    /// <summary>上一帧的关节角度备份（用于碰撞回滚）</summary>
    private float[] lastJointAngles = new float[6];

    /// <summary>上一帧碰撞检测的最小距离（用于判断移动方向）</summary>
    public float LastMinDistance { get; private set; } = float.MaxValue;

    /// <summary>当前帧是否发生了碰撞回滚</summary>
    public bool WasRollback { get; private set; } = false;

    /// <summary>碰撞回滚事件（供 UI 显示）</summary>
    public event Action OnRollback;

    public SimulationStateMachine()
    {
        states = new Dictionary<SimulationState, SimulationStateBase>
        {
            { SimulationState.Idle, new IdleState(this) },
            { SimulationState.Work, new WorkState(this) },
            { SimulationState.Succeed, new SucceedState(this) },
            { SimulationState.Fail, new FailState(this) },
            { SimulationState.Replay, new ReplayState(this) },
            { SimulationState.Joint, new JointState(this) },
            { SimulationState.TCP, new TCPState(this) }
        };
    }

    public bool TryChangeState(SimulationState target, SimulationContext ctx)
    {
        if (!IsTransitionAllowed(CurrentState, target)) return false;

        SimulationState prev = CurrentState;

        if (states.TryGetValue(prev, out var prevState))
        {
            try { prevState.Exit(ctx); }
            catch (Exception ex) { Debug.LogError($"State Exit error: {ex}"); }
        }

        CurrentState = target;

        if (states.TryGetValue(CurrentState, out var newState))
        {
            try { newState.Enter(ctx); }
            catch (Exception ex) { Debug.LogError($"State Enter error: {ex}"); }
        }

        OnStateChange?.Invoke(prev, CurrentState);
        return true;
    }

    /// <summary>
    /// 状态更新前：备份关节角度
    /// </summary>
    public void BackupJointAngles(SimulationContext ctx)
    {
        WasRollback = false;
        for (int i = 0; i < ctx.RobotModel.JointsCount; i++)
            lastJointAngles[i] = ctx.RobotModel.Joints[i].Angle;
    }

    public void Update(SimulationContext ctx, float dt)
    {
        if (states.TryGetValue(CurrentState, out var s))
        {
            try { s.Update(ctx, dt); }
            catch (Exception ex) { Debug.LogError($"State Update error: {ex}"); }
        }
    }

    public void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (states.TryGetValue(CurrentState, out var s))
        {
            try { s.HandleInput(ctx, key, num); }
            catch (Exception ex) { Debug.LogError($"State HandleInput error: {ex}"); }
        }
    }

    /// <summary>
    /// 回滚上一次关节移动（碰撞穿透时调用）
    /// </summary>
    public void RollbackLastMove(SimulationContext ctx)
    {
        ctx.RobotModel.SetJointAngles(lastJointAngles);
        WasRollback = true;
        OnRollback?.Invoke();
    }

    /// <summary>
    /// 更新上一帧距离记录
    /// </summary>
    public void UpdateLastDistance(float minDistance)
    {
        LastMinDistance = minDistance;
    }

    public SimulationStateBase GetCurrentStateInstance()
    {
        states.TryGetValue(CurrentState, out var s);
        return s;
    }

    private bool IsTransitionAllowed(SimulationState from, SimulationState to)
    {
        return from switch
        {
            SimulationState.Idle => to == SimulationState.Work || to == SimulationState.Joint || to == SimulationState.TCP,
            SimulationState.Joint or SimulationState.TCP => to == SimulationState.Idle,
            SimulationState.Succeed or SimulationState.Fail => to == SimulationState.Idle || to == SimulationState.Replay,
            SimulationState.Work => to == SimulationState.Idle || to == SimulationState.Succeed || to == SimulationState.Fail,
            SimulationState.Replay => to == SimulationState.Idle,
            _ => false,
        };
    }
}

/// <summary>
/// 仿真状态
/// </summary>
public enum SimulationState
{
    // 空闲，停止任何动作
    Idle = 1,
    // 工作，执行焊接任务
    Work = 2,
    // 成功，焊接任务成功
    Succeed = 4,
    // 失败，焊接任务失败
    Fail = 5,
    // 重播，回放上一次仿真的帧数据
    Replay = 6,
    // 关节控制，直接控制关节旋转
    Joint = 9,
    // TCP 控制，直接控制 TCP 平移
    TCP = 10
}
