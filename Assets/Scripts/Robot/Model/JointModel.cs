using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 关节模型，储存关节状态
/// </summary>
public class JointModel
{
    /// <summary>
    /// 关节角度（度）
    /// </summary>
    public float Angle { get; set; }

    /// <summary>
    /// 关节角速度（度/秒）
    /// </summary>
    public float AngleV { get; set; }

    /// <summary>
    /// 关节角加速度（度/秒^2）
    /// </summary>
    public float AngleA { get; set; }

    /// <summary>
    /// 关节坐标（米）（MDH 计算结果）
    /// </summary>
    public Vector3 Position { get; set; }

    /// <summary>
    /// 关节坐标（米）（Unity 得到）
    /// </summary>
    public Vector3 UPosition { get; set; }

    /// <summary>
    /// 该关节相对于前一关节的局部变换（MDH 计算结果）
    /// </summary>
    public Matrix4x4 LocalTransform { get; set; } = Matrix4x4.identity;

    /// <summary>
    /// 该关节在基坐标系中的世界变换矩阵（MDH 累乘结果）
    /// </summary>
    public Matrix4x4 WorldTransform { get; set; } = Matrix4x4.identity;
}
