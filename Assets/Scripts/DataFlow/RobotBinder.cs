using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 机器人绑定
/// </summary>
public class RobotBinder : MonoBehaviour
{
    public Transform[] JointTransforms = new Transform[6];
    public Transform Torch;
    public Transform Tcp;

    private RobotModel robot;

    public Vector3 RobotBasePosition;
    public Quaternion RobotBaseRotation;

    private Quaternion[] baseJointLocalRotations = new Quaternion[6];
    private Quaternion baseTCPRotation;

    public Transform[] Meshes = new Transform[7];
    public List<BoxCollider[]> BoxColliders = new(7);

    // 碰撞箱可视化相关
    private List<int> colliderVisualIds = new();
    private ColliderVisualizer colliderVisualizer;

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
            float targetAngle = -robot.Joints[i].Angle; // Unity是左手系所以取反
            Quaternion zRotation = Quaternion.Euler(0f, 0f, targetAngle);
            Quaternion baseLocal = (i < baseJointLocalRotations.Length) ? baseJointLocalRotations[i] : Quaternion.identity;
            jt.localRotation = baseLocal * zRotation;
        }

        UpdateUnityPose();
    }

    void Awake()
    {
        // 初始化碰撞体列表
        for (int i = 0; i < Meshes.Length; i++)
        {
            if (Meshes[i] != null)
            {
                var colliders = Meshes[i].GetComponents<BoxCollider>();
                BoxColliders.Add(colliders);
            }
            else
            {
                BoxColliders.Add(new BoxCollider[0]);
            }
        }
    }

    void Start()
    {
        if (robot == null)
        {
            Debug.LogWarning($"{nameof(RobotBinder)}: robot is not bound in Start()");
            return;
        }

        for (int i = 0; i < JointTransforms.Length; i++)
        {
            if (JointTransforms[i] != null)
            {
                baseJointLocalRotations[i] = JointTransforms[i].localRotation;
            }
        }

        if (Tcp != null)
        {
            if (robot.Config != null)
            {
                Tcp.localPosition = robot.Config.TCPOffset / 100f;
            }
            baseTCPRotation = Tcp.rotation;
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
        if (Tcp != null)
        {
            robot.UTCPPosition = GetUTCPPosition();
            robot.UTCPRotation = GetUTCPRotation();
        }
    }

    public Vector3 GetUTCPPosition()
    {
        if (Tcp == null) return Vector3.zero;
        // 获取 TCP 在 Unity 机器人坐标系中的位置
        return Quaternion.Inverse(RobotBaseRotation) * (Tcp.position - RobotBasePosition);
    }

    public Vector3 GetUJointPosition(int i)
    {
        if (JointTransforms == null || i < 0 || i >= JointTransforms.Length || JointTransforms[i] == null) return Vector3.zero;
        // 获取 Joint 在 Unity 机器人坐标系中的位置
        return Quaternion.Inverse(RobotBaseRotation) * (JointTransforms[i].position - RobotBasePosition);
    }

    public Quaternion GetUTCPRotation()
    {
        if (Tcp == null) return Quaternion.identity;
        // 获取 TCP 在 Unity 机器人坐标系中的旋转（相对于初始 baseTCPRotation）
        return Quaternion.Inverse(RobotBaseRotation) * Quaternion.Inverse(baseTCPRotation) * Tcp.rotation;
    }

    /// <summary>
    /// 添加所有碰撞箱到可视化器
    /// </summary>
    public void AddCollidersToVisualizer(ColliderVisualizer visualizer)
    {
        if (visualizer == null || BoxColliders == null) return;

        colliderVisualizer = visualizer;
        colliderVisualIds.Clear();

        // 机械臂碰撞箱用黄色
        Color armColor = Color.yellow;

        for (int i = 0; i < BoxColliders.Count; i++)
        {
            var colliders = BoxColliders[i];
            if (colliders == null) continue;

            // 使用 AddTracked 实现自动跟随（机械臂会运动）
            foreach (var boxCollider in colliders)
            {
                int id = visualizer.AddTracked(boxCollider, armColor);
                colliderVisualIds.Add(id);
            }
        }
    }

    /// <summary>
    /// 从可视化器中移除所有碰撞箱
    /// </summary>
    public void RemoveCollidersFromVisualizer()
    {
        if (colliderVisualizer == null) return;

        foreach (int id in colliderVisualIds)
        {
            colliderVisualizer.Remove(id);
        }
        colliderVisualIds.Clear();
    }
}
