using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 关节参数
/// </summary>
[System.Serializable]
public class JointParameters
{
    [Header("关节基本信息")]
    [Tooltip("关节名称")]
    public string JointName = "J";

    [Header("关节 D-H 参数")]
    [Tooltip("连杆扭角（度）")]
    public float Alpha;

    [Tooltip("连杆长度（米）")]
    public float A;

    [Tooltip("关节角度（度）")]
    public float Theta;

    [Tooltip("连杆偏移（米）")]
    public float D;

    [Header("关节限制")]
    [Tooltip("关节角最小值（度）")]
    public float AngleMin;

    [Tooltip("关节角最大值（度）")]
    public float AngleMax;

    [Tooltip("角速度最大值（度/秒）")]
    public float AngleVMax;
}
