using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 路径插值算法接口
/// </summary>
public interface PathInterpolation
{
    public float Evaluate(float t);
}

/// <summary>
/// 五次多项式插值
/// </summary>
public class QuinticPolynomial : PathInterpolation
{
    public void Build(float q0, float q1, float v0, float v1, float a0, float a1, float t)
    {

    }

    public float Evaluate(float t)
    {
        return 0f;
    }
}
