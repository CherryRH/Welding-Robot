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

    /// <summary>
    /// 计算点 p 到焊缝的几何误差（米）。
    /// 对于直线焊缝：点到直线的垂直距离，若投影超出线段范围则加上到最近端点的距离。
    /// 对于圆弧焊缝：径向误差，若角度超出弧段范围则加上最小角度差对应的弧长。
    /// </summary>
    public abstract float ComputePositionError(Vector3 p);

    /// <summary>
    /// 计算点 p 在焊缝上的投影位置（s 值，范围 [0,1]）。如果 p 不在焊缝附近，则返回最近端点的 s 值（0 或 1）。
    /// </summary>
    /// <param name="p"></param>
    /// <returns></returns>
    public abstract float ProjectPointToSeam(Vector3 p);

    /// <summary>
    /// 计算焊缝上某点的参考焊枪方向
    /// </summary>
    /// <param name="weldDirection">该点处的焊接方向（切向）</param>
    /// <returns>焊枪指向方向（从焊枪TCP指向焊点的单位向量）</returns>
    public virtual Vector3 ComputeGunDirection(Vector3 weldDirection)
    {
        Vector3 right = Vector3.Cross(weldDirection, Normal).normalized;
        float gunAngleRad = GunAngle * Mathf.Deg2Rad;
        Vector3 gunDir = -(Mathf.Cos(gunAngleRad) * right + Mathf.Sin(gunAngleRad) * Normal).normalized;
        return gunDir;
    }
}
