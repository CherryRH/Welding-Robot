using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 插值算法接口
/// </summary>
public interface Interpolation
{
    public float Evaluate(float t);
}

/// <summary>
/// 五次多项式插值（位置 + 速度 + 加速度约束）
/// </summary>
public class QuinticPolynomial : Interpolation
{
    // 多项式系数
    private float c0, c1, c2, c3, c4, c5;

    // 总时长
    private float duration;

    /// <summary>
    /// 构建五次多项式
    /// </summary>
    public void Build(float q0, float q1, float v0, float v1, float a0, float a1, float t)
    {
        duration = t;

        c0 = q0;
        c1 = v0;
        c2 = 0.5f * a0;

        float T = t;
        float T2 = T * T;
        float T3 = T2 * T;
        float T4 = T3 * T;
        float T5 = T4 * T;

        c3 = (20f * (q1 - q0)
            - (8f * v1 + 12f * v0) * T
            - (3f * a0 - a1) * T2) / (2f * T3);

        c4 = (30f * (q0 - q1)
            + (14f * v1 + 16f * v0) * T
            + (3f * a0 - 2f * a1) * T2) / (2f * T4);

        c5 = (12f * (q1 - q0)
            - (6f * v1 + 6f * v0) * T
            - (a0 - a1) * T2) / (2f * T5);
    }

    /// <summary>
    /// 计算 t 时刻的位置（t ∈ [0, T]）
    /// </summary>
    public float Evaluate(float t)
    {
        t = Mathf.Clamp(t, 0f, duration);

        float t2 = t * t;
        float t3 = t2 * t;
        float t4 = t3 * t;
        float t5 = t4 * t;

        return c0
             + c1 * t
             + c2 * t2
             + c3 * t3
             + c4 * t4
             + c5 * t5;
    }
}

/// <summary>
/// 线性插值
/// </summary>
public class LinearInterpolation : Interpolation
{
    private float start;
    private float end;
    private float duration;

    public void Build(float q0, float q1, float t)
    {
        start = q0;
        end = q1;
        duration = t;
    }

    public float Evaluate(float t)
    {
        if (duration <= 0f)
            return end;

        t = Mathf.Clamp(t, 0f, duration);
        float alpha = t / duration;

        return Mathf.Lerp(start, end, alpha);
    }
}
