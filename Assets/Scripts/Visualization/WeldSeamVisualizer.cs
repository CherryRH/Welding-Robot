using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.UI.Image;

/// <summary>
/// 焊缝可视化器
/// </summary>
public class WeldSeamVisualizer : MonoBehaviour
{
    public Transform WorkpieceOrigin;

    public GameObject SamplePoint;

    public float SamplePointShowScale = 0.1f;

    private List<GameObject> samplePoints = new();

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void ShowSeams(WeldSeamSampler sampler, float duration)
    {
        foreach (var geo in sampler.Geometries.Values)
        {
            int segments = 50;
            Vector3 prev = geo.GetPoint(0f);

            for (int i = 1; i <= segments; i++)
            {
                float s = geo.Length * i / segments;
                Vector3 cur = geo.GetPoint(s);

                Vector3 wp0 = WorkpieceOrigin.position + MathUtil.DataToUnityPosition(prev);
                Vector3 wp1 = WorkpieceOrigin.position + MathUtil.DataToUnityPosition(cur);

                Debug.DrawLine(wp0, wp1, Color.cyan, duration);

                prev = cur;
            }
        }
    }

    public void ShowSamplePoints(WeldSeamSampler sampler, float duration)
    {
        // 显示采样点
        if (sampler == null || SamplePoint == null || WorkpieceOrigin == null)
        {
            Debug.LogWarning("WorkpieceBinder: 参数未正确设置");
            return;
        }

        ClearSamplePoints();

        foreach (var kv in sampler.Samples)
        {
            var poses = kv.Value;
            if (poses == null || poses.Count == 0)
                continue;

            int count = poses.Count;
            int step = Mathf.Max(1, Mathf.RoundToInt(1f / SamplePointShowScale));

            for (int i = 0; i < count; i++)
            {
                // 首尾点必须显示
                bool forceShow = (i == 0 || i == count - 1);
                if (!forceShow && i % step != 0)
                    continue;

                Pose dataPose = poses[i];
                // 数据坐标 → Unity 坐标（相对工件原点）
                Vector3 unityPos = WorkpieceOrigin.position + MathUtil.DataToUnityPosition(dataPose.position);
                Quaternion unityRot = MathUtil.DataToUnityRotation(dataPose.rotation);

                Debug.DrawRay(
                    unityPos,
                    unityRot * Vector3.forward * 0.01f,
                    Color.red,
                    duration
                );

                // 实例化采样点
                GameObject point = Instantiate(
                    SamplePoint,
                    unityPos,
                    unityRot,
                    WorkpieceOrigin   // 跟随工件
                );
                point.name = $"SamplePoint_{kv.Key}_{i}";
                samplePoints.Add(point);
            }
        }
    }

    public void ClearSamplePoints()
    {
        // 清除采样点
        foreach (var point in samplePoints)
        {
            Destroy(point);
        }
        samplePoints.Clear();
    }
}
