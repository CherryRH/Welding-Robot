using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 机器人模型
/// </summary>
public class RobotModel
{
    public RobotConfig RobotConfig { get; private set; }

    public JointModel[] Joints { get; private set; }
    public int JointsCount => Joints.Length;

    /// <summary>
    /// TCP 位姿（MDH 计算）
    /// </summary>
    public Vector3 TCPPosition { get; set; }
    public Quaternion TCPRotation { get; set; }
    // 工具姿态欧拉角
    public Vector3 ToolEularAngles { get; set; }

    /// <summary>
    /// TCP 位姿（Unity 得到）
    /// </summary>
    public Vector3 UTCPPosition { get; set; }
    public Quaternion UTCPRotation { get; set; }

    public InverseKinematics IK { get; private set; } = new();

    public void Init(RobotConfig robotConfig)
    {
        // 初始化机器人模型
        RobotConfig = robotConfig;
        Joints = new JointModel[RobotConfig.JointsParameters.Length];
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
    }

    private void SetJointAngle(float angle, int index)
    {
        // 设定指定关节的角度并防止超出范围
        Joints[index].Angle = Mathf.Clamp(angle,
            RobotConfig.JointsParameters[index].AngleMin,
            RobotConfig.JointsParameters[index].AngleMax);
    }

    private void SetJointAngles(float[] angles)
    {
        // 设定所有关节的角度
        for (int i = 0; i < Joints.Length; i++)
        {
            SetJointAngle(angles[i], i);
        }
    }

    public void SingleJointRotationStep(float dt, int index, bool direction)
    {
        // 控制单个关节旋转步进
        float v = Mathf.Clamp(direction ?
            RobotConfig.TeleopAngleV :
            -RobotConfig.TeleopAngleV, -RobotConfig.JointsParameters[index].AngleVMax,
            RobotConfig.JointsParameters[index].AngleVMax);
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
            targetPos += dt * RobotConfig.TeleopTCPV * linearSpeed;
        }

        // 处理旋转（基于世界坐标系）
        if (angularSpeed.sqrMagnitude >= 1e-6f)
        {
            Vector3 axis = angularSpeed.normalized;
            float angle = dt * angularSpeed.magnitude * RobotConfig.TeleopTCPAngleV;
            Quaternion rotationIncrement = Quaternion.AngleAxis(angle, axis);
            targetRot = rotationIncrement * TCPRotation;
        }

        Pose targetPose = new(targetPos, targetRot);
        float[] solved = IK.Solve(targetPose, this);

        if (solved != null && solved.Length == Joints.Length)
        {
            SetJointAngles(solved);
        }
    }
}
