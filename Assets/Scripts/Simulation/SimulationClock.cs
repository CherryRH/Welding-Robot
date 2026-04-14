using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 仿真时钟
/// </summary>
public class SimulationClock
{
    public float FixedDeltaTime { get; private set; } = 0.01f;

    public bool IsRunning { get; private set; } = false;

    /// <summary>
    /// 当前仿真时间（秒）
    /// </summary>
    public float Time { get; private set; } = 0f;

    /// <summary>
    /// 是否已完成首次触发（t=0 时的预触发，用于建立参照）
    /// </summary>
    public bool HasTriggeredOnce { get; private set; } = false;

    private float accumulator = 0f;

    public SimulationClock(float fixedDt)
    {
        FixedDeltaTime = fixedDt;
    }

    public void Start()
    {
        IsRunning = true;
        accumulator = 0f;
    }

    public void Stop()
    {
        IsRunning = false;
        accumulator = 0f;
    }

    public void Reset()
    {
        IsRunning = false;
        Time = 0f;
        accumulator = 0f;
        HasTriggeredOnce = false;
    }

    /// <summary>
    /// 固定步长 Tick。当累计时间达到 FixedDeltaTime 时返回 true，表示需要执行一帧仿真。
    /// 
    /// 首次触发（HasTriggeredOnce = false）时，会提前到 t=0 时触发一次，
    /// 用于建立参照帧（不保存结果），之后恢复正常节奏（每 FixedDeltaTime 触发一次）。
    /// </summary>
    public bool Tick(float deltaTime)
    {
        // 首次触发：立即触发一次（t=0），但不推进时间
        if (!HasTriggeredOnce && IsRunning)
        {
            HasTriggeredOnce = true;
            return true;
        }

        accumulator += deltaTime;

        if (accumulator >= FixedDeltaTime)
        {
            accumulator -= FixedDeltaTime;
            if (IsRunning)
            {
                Time += FixedDeltaTime;
            }
            return true;
        }
        return false;
    }
}