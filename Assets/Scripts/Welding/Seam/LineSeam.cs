using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

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
        // 设置成员变量
        Id = data.ID;
        Name = data.Name;
        Speed = data.Speed;
        GunAngle = data.GunAngle;
        GunDistance = data.GunDistance;
        Normal = data.Normal.sqrMagnitude < 1e-6f ? Vector3.forward : data.Normal.normalized;
        StartPoint = data.StartPoint;
        EndPoint = data.EndPoint;
        direction = (EndPoint - StartPoint).normalized;
        LengthDeviation = data.LengthDeviation;
        // 计算几何长度
        Length = Vector3.Distance(StartPoint, EndPoint);
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
}
