using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊缝基类
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
    /// 焊缝总长度
    /// </summary>
    public float Length = 0f;

    /// <summary>
    /// 焊缝面法向（单位向量）
    /// </summary>
    public Vector3 Normal = Vector3.forward;

    /// <summary>
    /// 偏差值（米）
    /// </summary>
    public float LengthDeviation;

    /// <summary>
    /// 包角半径（米）：该焊缝结尾与下一条焊缝之间的圆角半径
    /// </summary>
    public float CornerRadius;

    /// <summary>
    /// 获取焊缝上 s (0,1) 对应的点
    /// </summary>
    public abstract Vector3 GetPoint(float s);

    /// <summary>
    /// 取得点切向（单位向量）
    /// </summary>
    public abstract Vector3 GetTangent(float s);

    /// <summary>
    /// 取得路径在 s 处的曲率
    /// </summary>
    public abstract float GetCurvature(float s);

    /// <summary>
    /// 判断点 p 是否位于该焊缝的有效范围内（几何判定）
    /// </summary>
    /// <param name="p">待检测点（世界坐标）</param>
    /// <param name="tol">距离容差（米）</param>
    /// <returns>p 在焊缝上返回 true</returns>
    public abstract bool ContainsPoint(Vector3 p, float tol = 1e-5f);
}
