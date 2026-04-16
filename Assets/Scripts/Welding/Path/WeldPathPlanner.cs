using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接路径规划器
/// </summary>
public class WeldPathPlanner
{
    /// <summary>
    /// 最大采样间隔（米）
    /// </summary>
    public float MaxInterval = 0.01f;

    /// <summary>
    /// 最小采样间隔（米）
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
        if (seam == null || seam.Length <= 0)
        {
            Debug.LogWarning("[Weld] Seam null or zero length, returning empty.");
            return points;
        }

        Debug.Log($"[Weld] Planning seam: length={seam.Length:F3}m");

        // 采样
        float s = 0f;
        List<float> samples = new() { s };
        while (s < 1f)
        {
            // 根据当前路径长度采样
            float currentCurvature = seam.GetCurvature(s);
            float interval = CalculateAdaptiveInterval(currentCurvature);

            // 确保 s 最终到达 1
            s = Mathf.Min(s + interval / seam.Length, 1f);

            // 添加到样本（如果 s 已经达到 1f 则不添加，等后面的 1f 检查处理）
            if (s < 1f)
                samples.Add(s);
        }

        // 确保终点被包含（如果最后一个样本不是 1f，则添加）
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

            points.Add(new TcpPathPoint(pose, WeldStateType.Weld, flag, seam, seam.Speed));
        }

        Debug.Log($"[Weld] Seam done: {points.Count} TCP points");
        return points;
    }

    /// <summary>
    /// 自适应采样间隔计算
    /// </summary>
    private float CalculateAdaptiveInterval(float curvature)
    {
        // 使用反比例映射：曲率越大，间隔越小
        float ds = MinInterval + (MaxInterval - MinInterval) / (1 + curvature * 1e-1f);

        return ds;
    }

    /// <summary>
    /// 计算焊枪TCP位置
    /// </summary>
    private Pose CalculateTcpPose(WeldSeam seam, Vector3 weldPoint, Vector3 weldDirection)
    {
        // 计算焊枪方向
        Vector3 right = Vector3.Cross(weldDirection, seam.Normal).normalized;
        float gunAngleRad = seam.GunAngle * Mathf.Deg2Rad;
        Vector3 gunDirection = -(Mathf.Cos(gunAngleRad) * right + Mathf.Sin(gunAngleRad) * seam.Normal).normalized;

        // 应用焊枪距离：焊枪位置 = 焊点 - 焊枪方向 * 距离
        Vector3 gunPosition = weldPoint - gunDirection * seam.GunDistance;
        // 转换到机器人坐标系
        gunPosition = robot.UserToRobot(gunPosition);

        // 计算焊枪旋转：Z轴指向焊枪方向，Y轴指向焊接方向
        Quaternion gunRotation = Quaternion.LookRotation(gunDirection, weldDirection);

        return new Pose(gunPosition, gunRotation);
    }
}