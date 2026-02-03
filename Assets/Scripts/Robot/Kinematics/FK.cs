using System;
using System.Reflection;
using UnityEngine;

/// <summary>
/// 正向运动学（使用 MDH 参数，矩阵顺序按 α -> a -> θ -> d）
/// </summary>
public static class FK
{
    public static Pose Compute(RobotModel robot)
    {
        if (robot == null) return new Pose(Vector3.zero, Quaternion.identity);
        if (robot.Config == null || robot.Config.JointsParameters == null) return new Pose(Vector3.zero, Quaternion.identity);

        Matrix4x4 T = Matrix4x4.identity;

        for (int i = 0; i < robot.Joints.Length; i++)
        {
            var jp = robot.Config.JointsParameters[i];

            float theta = robot.Joints[i].Angle + jp.Theta;

            Matrix4x4 Ti = MathUtil.MDH(jp.Alpha, jp.A, theta, jp.D);

            T *= Ti;

            robot.Joints[i].WorldTransform = T;
            robot.Joints[i].Position = new Vector3(T.m03, T.m13, T.m23);
        }

        // TCP 偏移
        Matrix4x4 tcp = Matrix4x4.Translate(robot.Config.TCPOffset);
        Matrix4x4 tcpT = T * tcp;
        robot.TCPTransform = tcpT;
        // 将 TCP 的法兰坐标系（J6）转换为工具坐标系
        Vector3 toolRotation = robot.Config.ToolRotation;
        Matrix4x4 flangeToTool = MathUtil.RotX(toolRotation.x) * MathUtil.RotY(toolRotation.y) * MathUtil.RotZ(toolRotation.z);
        Matrix4x4 toolT = tcpT * flangeToTool;

        Vector3 position = new(tcpT.m03, tcpT.m13, tcpT.m23);

        Quaternion rotation = MathUtil.RotationFromMatrix(tcpT);

        robot.TCPPosition = position;
        robot.TCPRotation = rotation;
        robot.ToolEularAngles = MathUtil.NormalizeEulerAngles(MathUtil.EulerZYX(toolT));

        return new Pose(position, rotation);
    }

    // -----------------------------
    // 帮助函数：基于给定角度计算 TCP pose（与 ForwardKinematics 相同的 MDH 顺序）
    // -----------------------------
    public static Pose ComputePoseFromAngles(RobotModel robot, float[] angles)
    {
        Matrix4x4 T = Matrix4x4.identity;
        int n = Math.Min(angles.Length, robot.Joints.Length);
        for (int i = 0; i < n; i++)
        {
            var jp = robot.Config.JointsParameters[i];
            float theta = angles[i] + jp.Theta;
            Matrix4x4 Ti = MathUtil.MDH(jp.Alpha, jp.A, theta, jp.D);
            T *= Ti;
        }

        Matrix4x4 tcp = Matrix4x4.Translate(robot.Config.TCPOffset);
        Matrix4x4 tcpT = T * tcp;
        Vector3 pos = new(tcpT.m03, tcpT.m13, tcpT.m23);
        Quaternion rot = MathUtil.RotationFromMatrix(tcpT);
        return new Pose(pos, rot);
    }

    // 计算第 i 个关节的变换矩阵，基于给定角度数组
    public static Matrix4x4 ComputeJointTransformMatrix(RobotModel robot, float[] angles, int jointIndex)
    {
        Matrix4x4 T = Matrix4x4.identity;
        for (int i = 0; i <= jointIndex; i++)
        {
            var jp = robot.Config.JointsParameters[i];
            float theta = angles[i] + jp.Theta;
            Matrix4x4 Ti = MathUtil.MDH(jp.Alpha, jp.A, theta, jp.D);
            T *= Ti;
        }
        return T;
    }

    // 计算第 i 个关节的世界位置（origin），基于给定角度数组
    public static Vector3 ComputeJointWorldPosition(RobotModel robot, float[] angles, int jointIndex)
    {
        Matrix4x4 T = Matrix4x4.identity;
        for (int i = 0; i <= jointIndex; i++)
        {
            var jp = robot.Config.JointsParameters[i];
            float theta = angles[i] + jp.Theta;
            Matrix4x4 Ti = MathUtil.MDH(jp.Alpha, jp.A, theta, jp.D);
            T *= Ti;
        }
        return new(T.m03, T.m13, T.m23);
    }

    // 计算第 i 个关节在世界坐标系下的 Z 方向（关节旋转轴方向）
    public static Vector3 ComputeJointZDirection(RobotModel robot, float[] angles, int jointIndex)
    {
        Matrix4x4 T = Matrix4x4.identity;
        for (int i = 0; i <= jointIndex; i++)
        {
            var jp = robot.Config.JointsParameters[i];
            float theta = angles[i] + jp.Theta;
            Matrix4x4 Ti = MathUtil.MDH(jp.Alpha, jp.A, theta, jp.D);
            T *= Ti;
        }
        // Z 轴方向为变换矩阵的第三列 (m02,m12,m22)
        Vector3 z = new Vector3(T.m02, T.m12, T.m22).normalized;
        if (z.sqrMagnitude < 1e-6f) z = Vector3.forward;
        return z;
    }
}
