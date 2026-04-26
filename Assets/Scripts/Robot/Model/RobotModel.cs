using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 机器人模型
/// </summary>
public class RobotModel
{
    public RobotConfig Config { get; private set; }

    public JointModel[] Joints { get; private set; }
    public float[] JointAngles { get => System.Array.ConvertAll(Joints, joint => joint.Angle); }
    public int JointsCount => Joints.Length;

    /// <summary>
    /// TCP 变换矩阵
    /// </summary>
    public Matrix4x4 TCPTransform { get; set; } = Matrix4x4.identity;

    /// <summary>
    /// TCP 位姿（MDH 计算）
    /// </summary>
    public Vector3 TCPPosition { get; set; }
    public Quaternion TCPRotation { get; set; }
    public Pose TCPPose { get => new Pose(TCPPosition, TCPRotation); }
    /// <summary>
    /// TCP 工具姿态欧拉角
    /// </summary>
    public Vector3 ToolEularAngles { get; set; }

    /// <summary>
    /// TCP 位姿（Unity 绑定后的世界坐标）
    /// </summary>
    public Vector3 UTCPPosition { get; set; }
    public Quaternion UTCPRotation { get; set; }
    public Pose UTCPPose { get => new Pose(UTCPPosition, UTCPRotation); }

    /// <summary>
    /// 用户坐标系偏移
    /// </summary>
    public Vector3 UserOffset { get; private set; } = Vector3.zero;

    public IK IK { get; private set; }

    // ============================================================
    // 实时数据（由 UpdateRealtimeData 统一更新）
    // ============================================================

    /// <summary>
    /// TCP 线速度（米/秒），原始前向差分值
    /// </summary>
    public Vector3 TcpVelocity { get; private set; }

    /// <summary>
    /// TCP 线速度平滑值（米/秒），一阶低通滤波
    /// </summary>
    public Vector3 SmoothedTcpVelocity { get; private set; }

    /// <summary>
    /// TCP 线速度标量（米/秒），= SmoothedTcpVelocity.magnitude
    /// </summary>
    public float TcpSpeed { get; private set; }

    /// <summary>
    /// 各关节角速度（弧度/秒），前向差分
    /// </summary>
    public float[] JointVelocities { get; private set; }

    /// <summary>
    /// 各关节角加速度（弧度/秒²），角速度的前向差分
    /// </summary>
    public float[] JointAccelerations { get; private set; }

    private Vector3 _prevTcpPos;
    private float[] _prevJointAngles;
    private float[] _prevJointVelocities;
    private float _prevTime = -1f;
    [SerializeField] private float _velocityAlpha = 0.3f;

    public void Init(RobotConfig robotConfig)
    {
        Config = robotConfig;
        Joints = new JointModel[Config.JointsParameters.Length];
        for (int i = 0; i < Joints.Length; i++)
        {
            Joints[i] = new JointModel { Angle = 0f, AngleV = 0f };
        }
        TCPPosition = Vector3.zero;
        TCPRotation = Quaternion.identity;
        ToolEularAngles = Vector3.zero;
        UTCPPosition = Vector3.zero;
        UTCPRotation = Quaternion.identity;
        IK = new IK(this);

        // 初始化实时数据缓冲
        _prevTcpPos = TCPPosition;
        _prevJointAngles = new float[Joints.Length];
        _prevJointVelocities = new float[Joints.Length];
        JointVelocities = new float[Joints.Length];
        JointAccelerations = new float[Joints.Length];

        // 用当前关节角度初始化历史缓冲
        System.Array.Copy(JointAngles, _prevJointAngles, Joints.Length);
        System.Array.Clear(_prevJointVelocities, 0, Joints.Length);
    }

    /// <summary>
    /// 设定用户坐标系偏移
    /// </summary>
    public void SetUserOffset(Vector3 offset)
    {
        UserOffset = offset;
    }

    /// <summary>
    /// 机器人坐标系坐标 → 用户坐标系
    /// </summary>
    public Vector3 RobotToUser(Vector3 robotPos)
    {
        return robotPos - UserOffset;
    }

    /// <summary>
    /// 用户坐标系坐标 → 机器人坐标系
    /// </summary>
    public Vector3 UserToRobot(Vector3 userPos)
    {
        return userPos + UserOffset;
    }

    /// <summary>
    /// 获取位姿的安全高度点
    /// </summary>
    public Pose GetSafePose(Pose pose)
    {
        Vector3 safePos = new(pose.position.x, pose.position.y, UserOffset.z + Config.TCPSafetyHeight);
        return new Pose(safePos, pose.rotation);
    }

    /// <summary>
    /// 设定指定关节的角度，超出范围自动钳位
    /// </summary>
    public void SetJointAngle(float angle, int index)
    {
        Joints[index].Angle = Mathf.Clamp(
            angle,
            Config.JointsParameters[index].AngleMin,
            Config.JointsParameters[index].AngleMax);
    }

    /// <summary>
    /// 设定所有关节角度
    /// </summary>
    public void SetJointAngles(float[] angles)
    {
        int n = Mathf.Min(angles.Length, Joints.Length);
        for (int i = 0; i < n; i++)
            SetJointAngle(angles[i], i);
    }

    /// <summary>
    /// 控制单个关节旋转步进
    /// </summary>
    public void SingleJointRotationStep(float dt, int index, bool direction)
    {
        float v = Mathf.Clamp(
            direction ? Config.TeleopAngleV : -Config.TeleopAngleV,
            -Config.JointsParameters[index].AngleVMax,
            Config.JointsParameters[index].AngleVMax);
        SetJointAngle(v * dt + Joints[index].Angle, index);
    }

    /// <summary>
    /// TCP 速度模式步进（同时处理平移和旋转）
    /// </summary>
    public void TCPMoveStep(float dt, Vector3 linearSpeed, Vector3 angularSpeed)
    {
        if (linearSpeed.sqrMagnitude < 1e-6f && angularSpeed.sqrMagnitude < 1e-6f)
            return;

        Vector3 targetPos = TCPPosition;
        Quaternion targetRot = TCPRotation;

        if (linearSpeed.sqrMagnitude >= 1e-6f)
            targetPos += dt * Config.TeleopTCPV * linearSpeed;

        if (angularSpeed.sqrMagnitude >= 1e-6f)
        {
            Vector3 axis = angularSpeed.normalized;
            float angle = dt * angularSpeed.magnitude * Config.TeleopTCPAngleV;
            targetRot = Quaternion.AngleAxis(angle, axis) * TCPRotation;
        }

        float[] solved = IK.Solve(new Pose(targetPos, targetRot), JointAngles);
        if (solved != null && solved.Length == Joints.Length)
            SetJointAngles(solved);
    }

    /// <summary>
    /// 判断给定时间间隔内关节角度变化是否超出速度限制
    /// </summary>
    public bool ViolatesJointVelocityLimit(float[] startJoints, float[] endJoints, float duration)
    {
        if (duration <= 1e-6f) return false;
        int n = Mathf.Min(startJoints.Length, endJoints.Length);
        for (int j = 0; j < n; j++)
        {
            float delta = Mathf.Abs(endJoints[j] - startJoints[j]);
            float requiredVelocity = delta / duration;
            float vmax = Config.JointsParameters[j].AngleVMax;
            if (requiredVelocity > vmax + 1e-4f) return true;
        }
        return false;
    }

    // ============================================================
    // 实时数据更新（每仿真步由 SimulationContext 调用一次）
    // ============================================================

    /// <summary>
    /// 统一更新所有实时数据（TCP 速度、关节角速度/角加速度）。
    /// dt 为固定仿真步长（秒），不依赖 Unity Time.deltaTime。
    /// </summary>
    public void UpdateRealtimeData(float currentTime, float dt)
    {
        // 防御：dt 过小或时间戳未推进时跳过，避免 NAN/Inf
        if (dt < 1e-6f || currentTime <= _prevTime)
        {
            return;
        }

        float[] currentAngles = JointAngles;

        // --- TCP 线速度（前向差分 + EMA 平滑）---
        Vector3 rawTcpVel = (TCPPosition - _prevTcpPos) / dt;
        TcpVelocity = rawTcpVel;
        SmoothedTcpVelocity = _velocityAlpha * rawTcpVel
                             + (1f - _velocityAlpha) * SmoothedTcpVelocity;
        TcpSpeed = SmoothedTcpVelocity.magnitude;
        
        // --- 关节角速度（前向差分）---
        for (int i = 0; i < JointsCount; i++)
        {
            float delta = currentAngles[i] - _prevJointAngles[i];
            float velocity = delta / dt;
            
            if (float.IsNaN(velocity) || float.IsInfinity(velocity))
            {
                velocity = 0f;
            }
            JointVelocities[i] = velocity;
        }

        // --- 关节角加速度（角速度的前向差分）---
        for (int i = 0; i < JointsCount; i++)
        {
            float deltaV = JointVelocities[i] - _prevJointVelocities[i];
            float acc = deltaV / dt;

            // 防御：角加速度跳变检测（停顿点或异常帧）
            if (MathUtil.FloatArrayMagnitude(_prevJointVelocities) < 1e-6f || float.IsNaN(acc) || float.IsInfinity(acc))
            {
                acc = 0f;
            }
            JointAccelerations[i] = acc;
        }

        // --- 保存本帧数据供下帧差分 ---
        _prevTcpPos = TCPPosition;
        System.Array.Copy(currentAngles, _prevJointAngles, JointsCount);
        System.Array.Copy(JointVelocities, _prevJointVelocities, JointsCount);
        _prevTime = currentTime;
    }

    /// <summary>
    /// 初始化历史缓冲，使下一帧差分基于当前状态。
    /// 在 t=0 预触发帧调用，避免第一帧产生异常差分值。
    /// </summary>
    public void InitHistoryBuffers()
    {
        _prevTime = -1f;
        _prevTcpPos = TCPPosition;
        if (JointAngles != null && _prevJointAngles != null)
        {
            System.Array.Copy(JointAngles, _prevJointAngles, JointsCount);
        }
        System.Array.Clear(_prevJointVelocities, 0, _prevJointVelocities.Length);
    }

    /// <summary>
    /// 重置实时数据状态（仿真开始前调用）
    /// </summary>
    public void ResetRealtimeData()
    {
        _prevTime = -1f;
        _prevTcpPos = TCPPosition;
        TcpVelocity = Vector3.zero;
        SmoothedTcpVelocity = Vector3.zero;
        TcpSpeed = 0f;
        System.Array.Clear(JointVelocities, 0, JointVelocities.Length);
        System.Array.Clear(JointAccelerations, 0, JointAccelerations.Length);
        System.Array.Clear(_prevJointVelocities, 0, _prevJointVelocities.Length);
    }
}