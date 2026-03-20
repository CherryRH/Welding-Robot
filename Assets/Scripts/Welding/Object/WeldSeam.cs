using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 描述焊缝对象
/// </summary>
public abstract class WeldSeam
{
    /// <summary>
    /// 焊缝ID
    /// </summary>
    public int Id = 0;

    /// <summary>
    /// 焊缝名称
    /// </summary>
    public string Name = "WeldSeam";

    /// <summary>
    /// 焊接线速度（米/秒）
    /// </summary>
    public float Speed = 0f;

    /// <summary>
    /// 焊枪角度（度）
    /// </summary>
    public float GunAngle = 90f;

    /// <summary>
    /// 焊枪距焊缝的距离（米）
    /// </summary>
    public float GunDistance = 0.01f;

    /// <summary>
    /// 起始点（米）
    /// </summary>
    public Vector3 StartPoint = Vector3.zero;

    /// <summary>
    /// 结束点（米）
    /// </summary>
    public Vector3 EndPoint = Vector3.zero;

    /// <summary>
    /// 几何总长度
    /// </summary>
    public float Length = 0f;

    /// <summary>
    /// 几何所在平面的法向（Line 可返回固定值）
    /// </summary>
    public Vector3 Normal = Vector3.forward;

    /// <summary>
    /// 偏差值（米）
    /// </summary>
    public float LengthDeviation;

    /// <summary>
    /// 焊缝参数化 s (0,1)
    /// </summary>
    public abstract Vector3 GetPoint(float s);

    /// <summary>
    /// 取切线方向（单位向量）
    /// </summary>
    public abstract Vector3 GetTangent(float s);

    /// <summary>
    /// 获取路径参数s处的曲率
    /// </summary>
    public abstract float GetCurvature(float s);
}
