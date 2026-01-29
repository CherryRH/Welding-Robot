using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "RobotConfig", menuName = "焊接配置/焊接机器人配置")]
public class RobotConfig : ScriptableObject
{
    [Header("机器人基本信息")]
    [Tooltip("机器人名称")]
    public string RobotName = "M10iD12";

    [Header("关节")]
    [Tooltip("关节参数")]
    public JointParameters[] JointsParameters = new JointParameters[6];

    [Header("末端执行器")]
    [Tooltip("末端执行器端点偏移（米）")]
    public Vector3 TCPOffset = Vector3.zero;
    [Tooltip("法兰盘相对于腕点偏移（米）")]
    public Vector3 FlangeOffset = Vector3.zero;
    [Tooltip("工具姿态旋转（度）")]
    public Vector3 ToolRotation = Vector3.zero;

    [Header("遥控")]
    [Tooltip("关节旋转角速度（度/秒）")]
    public float TeleopAngleV = 20f;
    [Tooltip("TCP 平移线速度（米/秒）")]
    public float TeleopTCPV = 0.2f;
    [Tooltip("TCP 姿态旋转角速度（度/秒）")]
    public float TeleopTCPAngleV = 20f;
}
