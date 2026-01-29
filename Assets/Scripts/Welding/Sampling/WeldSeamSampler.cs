using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊缝采样器（工件坐标系）
/// </summary>
public class WeldSeamSampler
{
    // 采样间隔
    private float sampleInterval = 0.001f;

    // 焊缝几何数据，Key为焊缝ID，Value为焊缝几何对象
    public Dictionary<int, WeldSeamGeometry> Geometries = new();

    // 采样结果，Key为焊缝ID，Value为焊枪TCP位姿列表
    public Dictionary<int, List<Pose>> Samples = new();

    /// <summary>
    /// 对焊接任务进行采样
    /// </summary>
    public void Sample(WeldTask task)
    {
        Geometries.Clear();
        Samples.Clear();

        foreach (var seam in task.WeldSeams)
        {
            WeldSeamGeometry geo = seam.Type switch
            {
                WeldSeam.WeldSeamType.Line => new LineGeometry(seam.StartPoint, seam.EndPoint, seam.Normal),

                WeldSeam.WeldSeamType.Arc => new ArcGeometry(seam.StartPoint, seam.MiddlePoints[0], seam.EndPoint, seam.Normal),

                _ => null
            };

            if (geo == null) continue;

            Geometries[seam.ID] = geo;
            Samples[seam.ID] = SampleGeometry(seam, geo);
        }
    }

    private List<Pose> SampleGeometry(WeldSeam seam, WeldSeamGeometry geo)
    {
        int count = Mathf.Max(2, Mathf.CeilToInt(geo.Length / sampleInterval) + 1);
        List<Pose> poses = new(count);

        for (int i = 0; i < count; i++)
        {
            float s = geo.Length * i / (count - 1);

            Vector3 p = geo.GetPoint(s);
            Vector3 t = geo.GetTangent(s);

            Pose pose = CalculateGunPose(seam, p, t, geo.Normal);
            poses.Add(pose);
        }

        return poses;
    }

    /// <summary>
    /// 计算焊枪TCP位姿
    /// </summary>
    private Pose CalculateGunPose(WeldSeam seam, Vector3 weldPoint, Vector3 weldDirection, Vector3 surfaceNormal)
    {
        if (surfaceNormal.sqrMagnitude < 1e-4f)
        {
            surfaceNormal = new(0, 0, 1);
        }

        // 计算焊枪方向
        Vector3 right = Vector3.Cross(weldDirection, surfaceNormal).normalized;
        float gunAngleRad = seam.GunAngle * Mathf.Deg2Rad;
        Vector3 gunDirection = -(Mathf.Cos(gunAngleRad) * right + Mathf.Sin(gunAngleRad) * surfaceNormal).normalized;

        // 应用焊枪距离：焊枪位置 = 焊接点 - 焊枪方向 * 距离
        Vector3 gunPosition = weldPoint - gunDirection * seam.GunDistance;

        // 计算焊枪旋转：Z轴指向焊枪方向，Y轴指向焊接方向
        Quaternion gunRotation = Quaternion.LookRotation(gunDirection, weldDirection);

        return new Pose(gunPosition, gunRotation);
    }

    /// <summary>
    /// 设置采样间隔
    /// </summary>
    public void SetSampleInterval(float interval)
    {
        sampleInterval = Mathf.Max(0.0001f, interval);
    }

    /// <summary>
    /// 获取指定焊缝的采样点数量
    /// </summary>
    public int GetSampleCount(int weldSeamID)
    {
        return Samples.ContainsKey(weldSeamID) ? Samples[weldSeamID].Count : 0;
    }

    /// <summary>
    /// 获取所有焊缝的采样点总数
    /// </summary>
    public int GetTotalSampleCount()
    {
        int total = 0;
        foreach (var samples in Samples.Values)
        {
            total += samples.Count;
        }
        return total;
    }
}
