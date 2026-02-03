using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// IK求解结果
/// </summary>
public class IKResult
{
    /// <summary>
    /// 多组可行的解
    /// </summary>
    public List<float[]> Solutions = new();

    public bool Success => Solutions.Count > 0;

    public float[] GetBestSolution(float[] current)
    {
        float minCost = float.MaxValue;
        float[] best = null;

        foreach (var sol in Solutions)
        {
            float cost = 0;
            for (int i = 0; i < 6; i++)
                cost += Mathf.Abs(sol[i] - current[i]);

            if (cost < minCost)
            {
                minCost = cost;
                best = sol;
            }
        }
        return best;
    }
}
