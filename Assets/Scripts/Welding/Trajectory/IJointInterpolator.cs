using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 关节角插值算法接口
/// </summary>
public interface IJointInterpolator
{
    public float[] Evaluate(float t);
}

/// <summary>
/// 线性插值
/// </summary>
public class LinearJointInterpolator : IJointInterpolator
{
    private float[] start;
    private float[] end;
    private float duration;
    private int dof;

    public void Build(float[] q0, float[] q1, float t)
    {
        dof = q0.Length;
        start = q0;
        end = q1;
        duration = t;
    }

    public float[] Evaluate(float t)
    {
        t = Mathf.Clamp(t, 0f, duration);
        float[] result = new float[dof];
        float alpha = t / duration;
        for (int i = 0; i < dof; i++)
        {
            result[i] = Mathf.Lerp(start[i], end[i], alpha);
        }
        return result;
    }
}

/// <summary>
/// 三次Hermite样条插值
/// </summary>
public class CubicHermiteSegmentInterpolator : IJointInterpolator
{
    private float[] a0;
    private float[] a1;
    private float[] a2;
    private float[] a3;

    private float duration;
    private int dof;

    public void Build(float[] q0, float[] q1, float[] v0, float[] v1, float segmentDuration)
    {
        duration = segmentDuration;
        dof = q0.Length;

        a0 = new float[dof];
        a1 = new float[dof];
        a2 = new float[dof];
        a3 = new float[dof];

        float T = duration;
        float T2 = T * T;
        float T3 = T2 * T;

        for (int i = 0; i < dof; i++)
        {
            a0[i] = q0[i];
            a1[i] = v0[i];
            a2[i] = (3f * (q1[i] - q0[i]) / T2) - (2f * v0[i] + v1[i]) / T;
            a3[i] = (-2f * (q1[i] - q0[i]) / T3) + (v0[i] + v1[i]) / T2;
        }
    }

    public float[] Evaluate(float t)
    {
        t = Mathf.Clamp(t, 0f, duration);

        float t2 = t * t;
        float t3 = t2 * t;

        float[] result = new float[dof];

        for (int i = 0; i < dof; i++)
        {
            result[i] = a0[i] + a1[i] * t + a2[i] * t2 + a3[i] * t3;
        }

        return result;
    }
}
