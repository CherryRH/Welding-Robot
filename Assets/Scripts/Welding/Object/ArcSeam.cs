using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using Unity.Burst.Intrinsics;
using UnityEngine;

/// <summary>
/// 圆弧焊缝
/// </summary>
public class ArcSeam : WeldSeam
{
    /// <summary>
    /// 圆心位置（米）
    /// </summary>
    public Vector3 Center;

    /// <summary>
    /// 半径（米）
    /// </summary>
    public float Radius;

    /// <summary>
    /// 角度（弧度，正负表示方向）
    /// </summary>
    public float Angle;

    /// <summary>
    /// 圆平面基向量（u: 起点方向, v: 垂直于u在圆平面内）
    /// </summary>
    public Vector3 u { get; private set; }
    public Vector3 v { get; private set; }

    public ArcSeam(WeldSeamData data)
    {
        if (data == null || data.Type != WeldSeamData.WeldSeamType.Arc || data.MiddlePoints.Count < 1) return;

        // 复制成员属性
        Id = data.ID;
        Name = data.Name;
        Speed = data.Speed;
        GunAngle = data.GunAngle;
        GunDistance = data.GunDistance;
        Normal = data.Normal.sqrMagnitude < 1e-6f ? Vector3.forward : data.Normal.normalized;
        StartPoint = data.StartPoint;
        EndPoint = data.EndPoint;
        LengthDeviation = data.LengthDeviation;
        CornerRadius = data.CornerRadius;

        // 从圆心、中间点、终点三点计算圆的参数
        Vector3 middle = data.MiddlePoints[0];
        Center = MathUtil.CalculateCircleCenter(StartPoint, middle, EndPoint);
        Radius = Vector3.Distance(StartPoint, Center);

        Vector3 v1 = (StartPoint - Center).normalized;
        Vector3 v2 = (EndPoint - Center).normalized;

        Normal = Vector3.Cross(middle - StartPoint, EndPoint - middle).normalized;

        if (Normal.sqrMagnitude < 1e-6f)
            Normal = Vector3.forward;
        // 保证法向朝向统一
        if (Normal.z < 0)
            Normal.z = -Normal.z;

        // 建立圆平面坐标系
        u = v1;
        v = Vector3.Cross(Normal, u).normalized;

        float rawAngle = Mathf.Atan2(
            Vector3.Dot(v2, v),
            Vector3.Dot(v2, u)
        );

        // 用 middle 点判断取哪一分支
        float midAngle = Mathf.Atan2(
            Vector3.Dot((middle - Center).normalized, v),
            Vector3.Dot((middle - Center).normalized, u)
        );

        // 确保中间点位于弧线上
        if (!MathUtil.IsAngleBetween(midAngle, 0f, rawAngle))
        {
            rawAngle = rawAngle > 0
                ? rawAngle - 2 * Mathf.PI
                : rawAngle + 2 * Mathf.PI;
        }

        Angle = rawAngle;
        Length = Mathf.Abs(Radius * Angle);
    }

    public ArcSeam(LineSeam line, int id, float r, Vector3 t1, Vector3 t2, Vector3 o, Vector3 m)
    {
        Id = id;
        Name = $"Arc_{id}";
        Speed = line.Speed;
        GunAngle = line.GunAngle;
        GunDistance = line.GunDistance;
        Normal = line.Normal;
        StartPoint = t1;
        EndPoint = t2;
        LengthDeviation = 0f;
        CornerRadius = 0f;

        // 从过渡圆弧计算圆的参数
        Vector3 middle = m;
        Center = o;
        Radius = r;

        Vector3 v1 = (StartPoint - Center).normalized;
        Vector3 v2 = (EndPoint - Center).normalized;

        Normal = Vector3.Cross(middle - StartPoint, EndPoint - middle).normalized;

        if (Normal.sqrMagnitude < 1e-6f)
            Normal = Vector3.forward;
        // 保证法向朝向统一
        if (Normal.z < 0)
            Normal.z = -Normal.z;

        // 建立圆平面坐标系
        u = v1;
        v = Vector3.Cross(Normal, u).normalized;

        float rawAngle = Mathf.Atan2(
            Vector3.Dot(v2, v),
            Vector3.Dot(v2, u)
        );

        // 用 middle 点判断取哪一分支
        float midAngle = Mathf.Atan2(
            Vector3.Dot((middle - Center).normalized, v),
            Vector3.Dot((middle - Center).normalized, u)
        );

        // 确保中间点位于弧线上
        if (!MathUtil.IsAngleBetween(midAngle, 0f, rawAngle))
        {
            rawAngle = rawAngle > 0
                ? rawAngle - 2 * Mathf.PI
                : rawAngle + 2 * Mathf.PI;
        }

        Angle = rawAngle;
        Length = Mathf.Abs(Radius * Angle);
    }

    public override Vector3 GetPoint(float s)
    {
        float angle = Angle * Mathf.Clamp01(s);
        return Center + Radius * (Mathf.Cos(angle) * u + Mathf.Sin(angle) * v);
    }

    public override Vector3 GetTangent(float s)
    {
        float angle = Angle * Mathf.Clamp01(s);

        Vector3 tangent =
            -Mathf.Sin(angle) * u +
             Mathf.Cos(angle) * v;

        if (Angle < 0)
            tangent = -tangent;

        return tangent.normalized;
    }

    public override float GetCurvature(float s)
    {
        return 1f / Radius;
    }

    public override bool ContainsPoint(Vector3 p, float tol = 1e-5f)
    {
        // 1. 必须在圆弧上（距圆心 ≈ Radius）
        if (Mathf.Abs(Vector3.Distance(p, Center) - Radius) > tol)
            return false;

        // 2. 角度必须在 [0, Angle] 范围内（允许端点容差）
        Vector3 d = (p - Center).normalized;
        float angle = Mathf.Atan2(Vector3.Dot(d, v), Vector3.Dot(d, u));

        // 角度容差（对应弧长容差）
        float angleTol = tol / Radius;

        if (Angle >= 0f)
            return angle >= -angleTol && angle <= Angle + angleTol;
        else
            return angle <= angleTol && angle >= Angle - angleTol;
    }

    public override float ProjectPointToSeam(Vector3 p)
    {
        // 投影到圆平面，再归一化到半径
        Vector3 toPoint = p - Center;
        Vector3 projected = toPoint.normalized * Radius;
        // 限制在弧段范围内
        float angle = Mathf.Atan2(Vector3.Dot(projected, v), Vector3.Dot(projected, u));
        // 根据 Angle 的符号正确限制范围
        if (Angle >= 0f)
            angle = Mathf.Clamp(angle, 0f, Angle);
        else
            angle = Mathf.Clamp(angle, Angle, 0f);
        return angle / Angle;
    }

    public override float ComputePositionError(Vector3 p)
    {
        // 1. 径向误差
        float radialError = Mathf.Abs(Vector3.Distance(p, Center) - Radius);

        // 2. 角度误差
        Vector3 d = (p - Center).normalized;
        float angle = Mathf.Atan2(Vector3.Dot(d, v), Vector3.Dot(d, u));

        // 将角度归一化到与 Angle 同符号的最小角度差
        float angleDiff = 0f;
        if (Angle >= 0f)
        {
            if (angle < 0f)
                angle += 2f * Mathf.PI;
            if (angle > Angle)
                angleDiff = angle - Angle;
            else if (angle < 0f)
                angleDiff = -angle;
        }
        else
        {
            if (angle > 0f)
                angle -= 2f * Mathf.PI;
            if (angle < Angle)
                angleDiff = Angle - angle;
            else if (angle > 0f)
                angleDiff = angle;
        }

        // 若角度在范围内，仅返回径向误差
        if (angleDiff <= 1e-6f)
            return radialError;

        // 若超出范围，加上角度差对应的弧长
        float arcLengthError = Mathf.Abs(angleDiff * Radius);
        return radialError + arcLengthError;
    }
}
