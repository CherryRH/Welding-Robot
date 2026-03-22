using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接路径点规划器
/// </summary>
public class WeldPathPlanner
{
    /// <summary>
    /// 最大间隔（米）
    /// </summary>
    public float MaxInterval = 0.01f;

    /// <summary>
    /// 最小间隔（米）
    /// </summary>
    public float MinInterval = 0.001f;

    private RobotModel robot;

    public void Init(RobotModel robot)
    {
        this.robot = robot;
    }

    public List<TcpPathPoint> Plan(WeldSeam seam)
    {
        List<TcpPathPoint> points = new();
        if (seam == null || seam.Length <= 0) return points;

        // 采样参数
        float s = 0f;
        List<float> samples = new() { s };
        while (s < 1f)
        {
            // 根据当前曲率计算间隔
            float currentCurvature = seam.GetCurvature(s);
            float interval = CalculateAdaptiveInterval(currentCurvature);

            // 确保 s 不超过 1
            s = Mathf.Min(s + interval / seam.Length, 1f);

            // 避免重复添加（如果 s 已经是 1f，跳过添加，由后面的确保语句处理）
            if (s < 1f)
                samples.Add(s);
        }

        // 确保末端点被采样（如果最后一个点不是 1f）
        if (samples.Count == 0 || samples[^1] < 1f)
            samples.Add(1f);

        // 生成 TCP 路径点
        for (int i = 0; i < samples.Count; i++)
        {
            s = samples[i];
            Vector3 p = seam.GetPoint(s);
            Vector3 t = seam.GetTangent(s);
            Pose pose = CalculateTcpPose(seam, p, t);

            TcpPathPoint.PointFlag flag = TcpPathPoint.PointFlag.Intermediate;
            if (i == 0) flag = TcpPathPoint.PointFlag.Start;
            else if (i == samples.Count - 1) flag = TcpPathPoint.PointFlag.End;

            points.Add(new TcpPathPoint(pose, TcpPathPoint.PointType.Weld, flag, seam, seam.Speed));
        }
        return points;
    }

    /// <summary>
    /// 计算自适应采样间隔
    /// </summary>
    private float CalculateAdaptiveInterval(float curvature)
    {
        // 使用非线性映射：曲率越大，间隔越小
        float ds = MinInterval + (MaxInterval - MinInterval) / (1 + curvature * 1e-1f);

        return ds;
    }

    /// <summary>
    /// 计算焊枪TCP位姿
    /// </summary>
    private Pose CalculateTcpPose(WeldSeam seam, Vector3 weldPoint, Vector3 weldDirection)
    {
        // 计算焊枪方向
        Vector3 right = Vector3.Cross(weldDirection, seam.Normal).normalized;
        float gunAngleRad = seam.GunAngle * Mathf.Deg2Rad;
        Vector3 gunDirection = -(Mathf.Cos(gunAngleRad) * right + Mathf.Sin(gunAngleRad) * seam.Normal).normalized;

        // 应用焊枪距离：焊枪位置 = 焊接点 - 焊枪方向 * 距离
        Vector3 gunPosition = weldPoint - gunDirection * seam.GunDistance;
        // 转化到基座坐标系
        gunPosition = robot.UserToRobot(gunPosition);

        // 计算焊枪旋转：Z轴指向焊枪方向，Y轴指向焊接方向
        Quaternion gunRotation = Quaternion.LookRotation(gunDirection, weldDirection);

        return new Pose(gunPosition, gunRotation);
    }
}
