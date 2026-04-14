using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 直线焊缝
/// </summary>
public class LineSeam : WeldSeam
{
    /// <summary>
    /// 直线方向（单位向量）
    /// </summary>
    private Vector3 direction;

    public LineSeam(WeldSeamData data)
    {
        if (data == null || data.Type != WeldSeamData.WeldSeamType.Line) return;
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
        Recalculate();
    }

    /// <summary>
    /// 根据当前 StartPoint / EndPoint 重算 Length 和 direction
    /// </summary>
    private void Recalculate()
    {
        Length = Vector3.Distance(StartPoint, EndPoint);
        direction = Length > 1e-6f
            ? (EndPoint - StartPoint).normalized
            : Vector3.zero;
    }

    /// <summary>
    /// 设置起点，重算 direction 和 Length
    /// </summary>
    public void SetStartPoint(Vector3 p)
    {
        StartPoint = p;
        Recalculate();
    }

    /// <summary>
    /// 设置终点，重算 direction 和 Length
    /// </summary>
    public void SetEndPoint(Vector3 p)
    {
        EndPoint = p;
        Recalculate();
    }

    public override Vector3 GetPoint(float s)
    {
        return StartPoint + Mathf.Clamp01(s) * Length * direction;
    }

    public override Vector3 GetTangent(float s)
    {
        return direction;
    }

    public override float GetCurvature(float s)
    {
        return 0f;
    }

    public override bool ContainsPoint(Vector3 p, float tol = 1e-5f)
    {
        // 投影到直线方向，计算 s 参数
        Vector3 diff = p - StartPoint;
        float s = Length > 1e-6f
            ? Vector3.Dot(diff, direction) / Length
            : 0f;

        // s 超出 [0,1] → 不在焊缝上
        if (s < 0f || s > 1f) return false;

        // 垂直方向距离 < tol
        float perpendicularDist = (diff - s * Length * direction).magnitude;
        return perpendicularDist < tol;
    }
}
