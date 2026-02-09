using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// TCP路径点
/// </summary>
public class TcpPathPoint
{
    /// <summary>
    /// TCP位姿（基座坐标系）
    /// </summary>
    public Pose Pose;

    /// <summary>
    /// 参考线速度（米/秒）
    /// </summary>
    public float Speed = 0.0f;

    /// <summary>
    /// 正在执行或接近的焊缝ID
    /// </summary>
    public int SeamId = 0;

    /// <summary>
    /// 路径点类型
    /// </summary>
    public enum PointType
    {
        Approach,
        Weld,
        Adjust
    }
    public PointType Type = PointType.Approach;

    /// <summary>
    /// 路径点标记
    /// </summary>
    public enum PointFlag
    {
        Start,
        Intermediate,
        End
    }
    public PointFlag Flag = PointFlag.Start;

    public TcpPathPoint(Pose pose, PointType type, PointFlag flag, int seamId, float speed = 0f)
    {
        Pose = pose;
        Type = type;
        Flag = flag;
        SeamId = seamId;
        Speed = speed;
    }
}
