using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
/// 弧线焊缝
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
    /// 角度，带正负（度）
    /// </summary>
    public float Angle;

    /// <summary>
    /// 圆平面基向量
    /// </summary>
    private Vector3 u;
    private Vector3 v;

    public ArcSeam(WeldSeamData data)
    {
        if (data == null || data.Type != WeldSeamData.WeldSeamType.Arc || data.MiddlePoints.Count < 1) return;
        // 设置成员变量
        Id = data.ID;
        Name = data.Name;
        Speed = data.Speed;
        GunAngle = data.GunAngle;
        GunDistance = data.GunDistance;
        Normal = data.Normal.sqrMagnitude < 1e-6f ? Vector3.forward : data.Normal.normalized;
        StartPoint = data.StartPoint;
        EndPoint = data.EndPoint;
        LengthDeviation = data.LengthDeviation;
        // 计算圆心、半径、角度、法向量等几何参数
        Vector3 middle = data.MiddlePoints[0];
        Center = MathUtil.CalculateCircleCenter(StartPoint, middle, EndPoint);
        Radius = Vector3.Distance(StartPoint, Center);

        Vector3 v1 = (StartPoint - Center).normalized;
        Vector3 v2 = (EndPoint - Center).normalized;

        Normal = Vector3.Cross(middle - StartPoint, EndPoint - middle).normalized;

        if (Normal.sqrMagnitude < 1e-6f)
            Normal = Vector3.forward;
        // 保持法向向上
        if (Normal.z < 0)
            Normal.z = -Normal.z;

        // 构建圆平面坐标系
        u = v1;
        v = Vector3.Cross(Normal, u).normalized;

        float rawAngle = Mathf.Atan2(
            Vector3.Dot(v2, v),
            Vector3.Dot(v2, u)
        );

        // 用 middle 点判断取哪一段弧
        float midAngle = Mathf.Atan2(
            Vector3.Dot((middle - Center).normalized, v),
            Vector3.Dot((middle - Center).normalized, u)
        );

        // 确保中间点落在弧段上
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
}
