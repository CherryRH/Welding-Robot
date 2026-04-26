using UnityEngine;

/// <summary>
/// 仿真结果帧 — 记录一个仿真时刻的所有实时数据。
/// 按时间戳存储在 WeldResultData.Frames 中。
/// </summary>
[System.Serializable]
public class WeldResultFrame
{
    /// <summary>
    /// 仿真时间（秒）
    /// </summary>
    public float Timestamp;

    /// <summary>
    /// TCP 位置（MDH 机器人坐标系，米）
    /// </summary>
    public Vector3 TcpPosition;

    /// <summary>
    /// TCP 线速度向量（米/秒）
    /// </summary>
    public Vector3 TcpVelocity;

    /// <summary>
    /// TCP 线速度标量（米/秒），= magnitude of TcpVelocity
    /// </summary>
    public float TcpSpeed;

    /// <summary>
    /// 各关节角（弧度），顺序 J0~J5
    /// </summary>
    public float[] JointAngles;

    /// <summary>
    /// 各关节角速度（弧度/秒），前向差分
    /// </summary>
    public float[] JointVelocities;

    /// <summary>
    /// 各关节角加速度（弧度/秒²），角速度的前向差分
    /// </summary>
    public float[] JointAccelerations;

    /// <summary>
    /// 当前轨迹点类型（Approach / Weld / Adjust）
    /// </summary>
    public WeldStateType PointType;

    /// <summary>
    /// 所属焊缝 ID（-1 表示不属于任何焊缝）
    /// </summary>
    public int SeamId;

    /// <summary>
    /// 线速度误差（米/秒）= 当前速度 - 参考速度，不取绝对值
    /// </summary>
    public float SpeedError;

    /// <summary>
    /// 焊点位置误差（米）：点到焊缝的几何距离
    /// </summary>
    public float PositionError;

    /// <summary>
    /// 焊枪姿态误差（度）：当前焊枪方向与参考方向的夹角
    /// </summary>
    public float OrientationError;

    public WeldResultFrame(
        float timestamp,
        Vector3 tcpPos,
        Vector3 tcpVel,
        float[] jointAngles,
        float[] jointVelocities,
        float[] jointAccelerations,
        WeldStateType segmentType = WeldStateType.Approach,
        int seamId = -1,
        float speedError = 0f,
        float positionError = 0f,
        float orientationError = 0f)
    {
        Timestamp = timestamp;
        TcpPosition = tcpPos;
        TcpVelocity = tcpVel;
        TcpSpeed = tcpVel.magnitude;
        JointAngles = (float[])jointAngles.Clone();
        JointVelocities = (float[])jointVelocities.Clone();
        JointAccelerations = (float[])jointAccelerations.Clone();
        PointType = segmentType;
        SeamId = seamId;
        SpeedError = speedError;
        PositionError = positionError;
        OrientationError = orientationError;
    }
}