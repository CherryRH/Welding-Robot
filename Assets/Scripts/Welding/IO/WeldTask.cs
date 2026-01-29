using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 描述焊接任务数据
/// </summary>
[System.Serializable]
public class WeldTask
{
    /// <summary>
    /// 任务名称
    /// </summary>
    public string TaskName;

    /// <summary>
    /// 工件模型文件名称
    /// （暂设为默认值，暂时不进行模型动态加载）
    /// </summary>
    public string WorkpieceFileName;

    /// <summary>
    /// 焊缝数据列表
    /// </summary>
    public List<WeldSeam> WeldSeams;
}

/// <summary>
/// 描述焊缝数据
/// </summary>
[System.Serializable]
public class WeldSeam
{
    /// <summary>
    /// 焊缝类型
    /// </summary>
    public enum WeldSeamType
    {
        Line,
        Arc
    }
    public WeldSeamType Type;

    /// <summary>
    /// 焊缝ID
    /// </summary>
    public int ID;

    /// <summary>
    /// 焊缝名称
    /// </summary>
    public string Name;

    /// <summary>
    /// 焊接线速度（米/秒）
    /// </summary>
    public float Speed;

    /// <summary>
    /// 焊枪角度（度）
    /// </summary>
    public float GunAngle;

    /// <summary>
    /// 焊枪距焊缝的距离（米）
    /// </summary>
    public float GunDistance;

    /// <summary>
    /// 起始点（米）
    /// </summary>
    public Vector3 StartPoint;

    /// <summary>
    /// 结束点（米）
    /// </summary>
    public Vector3 EndPoint;

    /// <summary>
    /// 中间点，可选（米）
    /// </summary>
    public List<Vector3> MiddlePoints;

    /// <summary>
    /// 焊接面法向（单位向量）
    /// </summary>
    public Vector3 Normal;

    /// <summary>
    /// 偏差值（米）
    /// </summary>
    public float LengthDeviation;

    /// <summary>
    /// 与下一个焊缝是否连续
    /// </summary>
    public bool IsContinous;
}
