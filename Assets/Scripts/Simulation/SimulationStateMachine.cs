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

    public SimulationStateMachine()
    {
        states = new Dictionary<SimulationState, SimulationStateBase>
        {
            { SimulationState.Idle, new IdleState(this) },
            { SimulationState.Work, new WorkState(this) },
            { SimulationState.Pause, new PauseState(this) },
            { SimulationState.Succeed, new SucceedState(this) },
            { SimulationState.Fail, new FailState(this) },
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
            SimulationState.Joint or SimulationState.TCP or SimulationState.Succeed or SimulationState.Fail => to == SimulationState.Idle,
            SimulationState.Work => to == SimulationState.Pause || to == SimulationState.Succeed || to == SimulationState.Fail,
            SimulationState.Pause => to == SimulationState.Work,
            _ => false,
        };
    }
}

/// <summary>
/// 仿真状态
/// </summary>
public enum SimulationState
{
    // 待机（停止任何动作）
    Idle = 1,
    // 工作（执行焊接任务）
    Work = 2,
    // 暂停（焊接任务仿真暂停）
    Pause = 3,
    // 成功（焊接任务仿真成功）
    Succeed = 4,
    // 失败（焊接任务仿真失败）
    Fail = 5,
    // 关节控制（直接控制关节旋转）
    Joint = 9,
    // TCP 控制（直接控制 TCP 平移）
    TCP = 10
}