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

    public float Time { get; private set; } = 0f;

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
    }

    /// <summary>
    /// 视觉/节流用 Tick：无论时钟是否运行都累积并在达到 FixedDeltaTime 时返回 true。
    /// 当 IsRunning 为 true 时，会同时推进 Time（仿真时间）；否则仅作为视觉节拍使用（不改变 Time）。
    /// </summary>
    public bool Tick(float deltaTime)
    {
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
