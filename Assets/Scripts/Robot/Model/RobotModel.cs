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
    /// TCP变换矩阵
    /// </summary>
    public Matrix4x4 TCPTransform { get; set; } = Matrix4x4.identity;

    /// <summary>
    /// TCP 位姿（MDH 计算）
    /// </summary>
    public Vector3 TCPPosition { get; set; }
    public Quaternion TCPRotation { get; set; }
    public Pose TCPPose { get => new(TCPPosition, TCPRotation); }
    // 工具姿态欧拉角
    public Vector3 ToolEularAngles { get; set; }

    /// <summary>
    /// TCP 位姿（Unity 得到）
    /// </summary>
    public Vector3 UTCPPosition { get; set; }
    public Quaternion UTCPRotation { get; set; }
    public Pose UTCPPose { get => new(UTCPPosition, UTCPRotation); }

    // 用户坐标系偏移
    public Vector3 UserOffset { get; private set; } = Vector3.zero;

    public IK IK { get; private set; }

    public void Init(RobotConfig robotConfig)
    {
        // 初始化机器人模型
        Config = robotConfig;
        Joints = new JointModel[Config.JointsParameters.Length];
        for (int i = 0; i < Joints.Length; i++)
        {
            Joints[i] = new()
            {
                Angle = 0f,
                AngleV = 0f
            };
        }

        TCPPosition = Vector3.zero;
        TCPRotation = Quaternion.identity;
        ToolEularAngles = Vector3.zero;
        UTCPPosition = Vector3.zero;
        UTCPRotation = Quaternion.identity;

        IK = new(this);
    }

    public void SetUserOffset(Vector3 offset)
    {
        // 设定用户坐标系偏移
        UserOffset = offset;
    }

    public Vector3 RobotToUser(Vector3 robotPos)
    {
        return robotPos - UserOffset;
    }

    public Vector3 UserToRobot(Vector3 userPos)
    {
        return userPos + UserOffset;
    }

    public Pose GetSafePose(Pose pose)
    {
        // 获取位姿的安全高度点
        Vector3 pos = pose.position;
        Vector3 safePos = new(pos.x, pos.y, UserOffset.z + Config.TCPSafetyHeight);
        return new Pose(safePos, pose.rotation);
    }

    public void SetJointAngle(float angle, int index)
    {
        // 设定指定关节的角度并防止超出范围
        Joints[index].Angle = Mathf.Clamp(angle,
            Config.JointsParameters[index].AngleMin,
            Config.JointsParameters[index].AngleMax);
    }

    public void SetJointAngles(float[] angles)
    {
        int n = Mathf.Min(angles.Length, Joints.Length);
        // 设定所有关节的角度
        for (int i = 0; i < n; i++)
        {
            SetJointAngle(angles[i], i);
        }
    }

    public void SingleJointRotationStep(float dt, int index, bool direction)
    {
        // 控制单个关节旋转步进
        float v = Mathf.Clamp(direction ?
            Config.TeleopAngleV :
            -Config.TeleopAngleV, -Config.JointsParameters[index].AngleVMax,
            Config.JointsParameters[index].AngleVMax);
        SetJointAngle(v * dt + Joints[index].Angle, index);
    }

    public void TCPMoveStep(float dt, Vector3 linearSpeed, Vector3 angularSpeed)
    {
        // 同时处理平移和旋转
        if (linearSpeed.sqrMagnitude < 1e-6f && angularSpeed.sqrMagnitude < 1e-6f) return;

        Vector3 targetPos = TCPPosition;
        Quaternion targetRot = TCPRotation;

        // 处理平移（基于世界坐标系）
        if (linearSpeed.sqrMagnitude >= 1e-6f)
        {
            targetPos += dt * Config.TeleopTCPV * linearSpeed;
        }

        // 处理旋转（基于世界坐标系）
        if (angularSpeed.sqrMagnitude >= 1e-6f)
        {
            Vector3 axis = angularSpeed.normalized;
            float angle = dt * angularSpeed.magnitude * Config.TeleopTCPAngleV;
            Quaternion rotationIncrement = Quaternion.AngleAxis(angle, axis);
            targetRot = rotationIncrement * TCPRotation;
        }

        Pose targetPose = new(targetPos, targetRot);
        float[] solved = IK.Solve(targetPose, JointAngles);

        if (solved != null && solved.Length == Joints.Length)
        {
            SetJointAngles(solved);
        }
    }

    public bool ViolatesJointVelocityLimit(float[] startJoints, float[] endJoints, float duration)
    {
        // 判断是否超出关节角速度限制
        if (duration <= 1e-6f) return false;

        int n = Mathf.Min(startJoints.Length, endJoints.Length);
        for (int j = 0; j < n; j++)
        {
            float delta = Mathf.Abs(endJoints[j] - startJoints[j]);
            float requiredVelocity = delta / duration;
            float vmax = Config.JointsParameters[j].AngleVMax;
            if (requiredVelocity > vmax + 1e-4f)
            {
                return true;
            }
        }
        return false;
    }
}
