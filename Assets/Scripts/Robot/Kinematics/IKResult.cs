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

    public float[] GetBestSolution(float[] current, RobotModel robot)
    {
        float minCost = float.MaxValue;
        float[] best = null;

        foreach (var sol in Solutions)
        {
            float[] adjusted = (float[])sol.Clone();

            // ====== J6 多圈补偿 ======
            float minRange = robot.Config.JointsParameters[5].AngleMin;
            float maxRange = robot.Config.JointsParameters[5].AngleMax;

            float bestJ6 = adjusted[5];
            float minDiff = float.MaxValue;

            // 尝试 ±2 圈（足够覆盖）
            for (int k = -2; k <= 2; k++)
            {
                float candidate = sol[5] + 360f * k;

                if (candidate < minRange || candidate > maxRange)
                    continue;

                float diff = Mathf.Abs(candidate - current[5]);

                if (diff < minDiff)
                {
                    minDiff = diff;
                    bestJ6 = candidate;
                }
            }

            adjusted[5] = bestJ6;

            // ====== 计算整体 cost ======
            float cost = 0f;

            // 关节角度差
            for (int i = 0; i < 6; i++)
                cost += Mathf.Abs(adjusted[i] - current[i]);

            // 限制Roll
            cost += Mathf.Abs(adjusted[3] + adjusted[5] - current[3] - current[5]);

            if (cost < minCost)
            {
                minCost = cost;
                best = adjusted;
            }
        }

        return best;
    }
}
