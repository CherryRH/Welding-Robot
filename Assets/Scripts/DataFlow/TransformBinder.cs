using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 机器人姿态绑定
/// </summary>
public class TransformBinder : MonoBehaviour
{
    public Transform[] JointTransforms = new Transform[6];
    public Transform TCP;

    public float[] JointAngleMultiple = new float[6] { 1, 1, 1, 1, 1, 1 };

    private RobotModel robot;

    public Vector3 RobotBasePosition;
    public Quaternion RobotBaseRotation;

    private Quaternion[] baseJointLocalRotations = new Quaternion[6];
    private Quaternion baseTCPRotation;

    public void Bind(RobotModel model)
    {
        robot = model;
    }

    public void Apply()
    {
        if (robot == null) return;

        // 在视觉节拍到达时更新基础位姿
        RobotBasePosition = transform.position;
        RobotBaseRotation = transform.rotation;

        // 应用所有关节的角度并更新相关数据（并在同一节拍更新模型数据）
        for (int i = 0; i < JointTransforms.Length; i++)
        {
            var jt = JointTransforms[i];
            if (jt == null) continue;

            // 角度应用
            float multiple = (JointAngleMultiple != null && i < JointAngleMultiple.Length) ? JointAngleMultiple[i] : 1f;
            float targetAngle = robot.Joints[i].Angle * multiple;
            Quaternion zRotation = Quaternion.Euler(0f, 0f, targetAngle);
            Quaternion baseLocal = (i < baseJointLocalRotations.Length) ? baseJointLocalRotations[i] : Quaternion.identity;
            jt.localRotation = baseLocal * zRotation;
        }

        UpdateUnityPose();
    }

    void Awake()
    {
        
    }

    void Start()
    {
        if (robot == null)
        {
            Debug.LogWarning($"{nameof(TransformBinder)}: robot is not bound in Start()");
            return;
        }

        for (int i = 0; i < JointTransforms.Length; i++)
        {
            if (JointTransforms[i] != null)
            {
                baseJointLocalRotations[i] = JointTransforms[i].localRotation;
            }
        }

        if (TCP != null)
        {
            if (robot.RobotConfig != null)
            {
                TCP.localPosition = robot.RobotConfig.TCPOffset / 100f;
            }
            baseTCPRotation = TCP.rotation;
        }

        // 初始化基座
        RobotBasePosition = transform.position;
        RobotBaseRotation = transform.rotation;

        UpdateUnityPose();
    }

    void Update()
    {
        // 现在主要的数据更新由 Apply() 在视觉节拍处触发。
        // 如需每帧额外的逻辑可放这里；为空以避免重复更新。
    }

    private void UpdateUnityPose()
    {
        // 更新机器人模型中的 Unity 位姿
        for (int i = 0; i < JointTransforms.Length; i++)
        {
            if (JointTransforms[i] != null)
            {
                robot.Joints[i].UPosition = GetUJointPosition(i);
            }
        }
        if (TCP != null)
        {
            robot.UTCPPosition = GetUTCPPosition();
            robot.UTCPRotation = GetUTCPRotation();
        }
    }

    public Vector3 GetUTCPPosition()
    {
        if (TCP == null) return Vector3.zero;
        // 获取 TCP 在 Unity 机器人坐标系中的位置
        return Quaternion.Inverse(RobotBaseRotation) * (TCP.position - RobotBasePosition);
    }

    public Vector3 GetUJointPosition(int i)
    {
        if (JointTransforms == null || i < 0 || i >= JointTransforms.Length || JointTransforms[i] == null) return Vector3.zero;
        // 获取 Joint 在 Unity 机器人坐标系中的位置
        return Quaternion.Inverse(RobotBaseRotation) * (JointTransforms[i].position - RobotBasePosition);
    }

    public Quaternion GetUTCPRotation()
    {
        if (TCP == null) return Quaternion.identity;
        // 获取 TCP 在 Unity 机器人坐标系中的旋转（相对于初始 baseTCPRotation）
        return Quaternion.Inverse(RobotBaseRotation) * Quaternion.Inverse(baseTCPRotation) * TCP.rotation;
    }
}
