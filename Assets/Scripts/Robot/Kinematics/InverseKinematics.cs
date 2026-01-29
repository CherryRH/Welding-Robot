using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 逆向运动学（CCD, JT）
/// </summary>
public class InverseKinematics
{
    public enum IKMethod
    {
        CCD,    // Cyclic Coordinate Descent
        JT      // Jacobian Transpose
    }

    public IKMethod Method = IKMethod.CCD;

    // 公共参数，可按需调整
    public int MaxIterations = 100;
    public float PositionTolerance = 1e-3f; // 单位：米（与 TCP Position 单位一致）
    public float CCDStepLimitDeg = 5f; // CCD 单次关节调整限制（度）
    public float JacobianStep = 1.0f; // Jacobian Transpose 步长系数（调小以保证收敛）
    public float JacobianDeltaDeg = 1e-1f; // 数值雅可比扰动（度）
    public float JacobianStepLimitDeg = 2f; // Jacobian 单次关节调整限制（度）

    public float[] Solve(Pose target, RobotModel robot)
    {
        if (robot == null || robot.RobotConfig == null || robot.RobotConfig.JointsParameters == null)
            return new float[robot?.Joints.Length ?? 0];

        switch (Method)
        {
            case IKMethod.CCD:
                return SolveCCD(target, robot);
            case IKMethod.JT:
                return SolveJacobianTranspose(target, robot);
            default:
                return SolveCCD(target, robot);
        }
    }

    // -----------------------------
    // CCD 实现（位置控制）
    // -----------------------------
    private float[] SolveCCD(Pose target, RobotModel robot)
    {
        int n = robot.Joints.Length;
        var angles = new float[n];
        for (int i = 0; i < n; i++) angles[i] = robot.Joints[i].Angle;

        for (int iter = 0; iter < MaxIterations; iter++)
        {
            // 计算当前 TCP
            Pose current = ForwardKinematics.ComputePoseFromAngles(robot, angles);
            Vector3 tcpPos = current.position;
            if ((tcpPos - target.position).sqrMagnitude <= PositionTolerance * PositionTolerance)
                break;

            // 从末端到基座逐关节调整
            for (int i = n - 1; i >= 0; i--)
            {
                // 计算第 i 个关节的世界位置（origin of joint frame）
                Vector3 jointPos = ForwardKinematics.ComputeJointWorldPosition(robot, angles, i);

                Vector3 toCurr = tcpPos - jointPos;
                Vector3 toTarget = target.position - jointPos;

                if (toCurr.sqrMagnitude < 1e-8f || toTarget.sqrMagnitude < 1e-8f)
                    continue;

                toCurr.Normalize();
                toTarget.Normalize();

                float cosAngle = Mathf.Clamp(Vector3.Dot(toCurr, toTarget), -1f, 1f);
                float angleBetweenDeg = Mathf.Acos(cosAngle) * Mathf.Rad2Deg;
                if (angleBetweenDeg < 1e-4f) continue;

                // 旋转方向由 cross product 决定（world frame）
                Vector3 cross = Vector3.Cross(toCurr, toTarget);
                // 在 MDH 中关节是绕自身 Z 轴旋转；我们用 cross 的 Z 分量（project to joint's rotation axis）
                // 为简单实现，我们用 cross 在世界坐标的方向与 joint 的 z 方向点乘判断符号。
                // 计算 joint z 方向（world）
                Vector3 jointZ = ForwardKinematics.ComputeJointZDirection(robot, angles, i);
                float sign = Mathf.Sign(Vector3.Dot(jointZ, cross));
                if (sign == 0) sign = 1f;

                float deltaDeg = Mathf.Clamp(0.5f * angleBetweenDeg * sign, -CCDStepLimitDeg, CCDStepLimitDeg); // 加入角度缩放

                angles[i] = Mathf.Clamp(angles[i] + deltaDeg,
                    robot.RobotConfig.JointsParameters[i].AngleMin,
                    robot.RobotConfig.JointsParameters[i].AngleMax);

                // 更新当前 TCP（局部更新即可）
                current = ForwardKinematics.ComputePoseFromAngles(robot, angles);
                tcpPos = current.position;

                if ((tcpPos - target.position).sqrMagnitude <= PositionTolerance * PositionTolerance)
                    break;
            }
        }

        return angles;
    }

    // -----------------------------
    // Jacobian Transpose（数值雅可比）
    // -----------------------------
    private float[] SolveJacobianTranspose(Pose target, RobotModel robot)
    {
        int n = robot.Joints.Length;
        var angles = new float[n];
        for (int i = 0; i < n; i++)
            angles[i] = robot.Joints[i].Angle;

        const float oriWeight = 0f;          // 姿态误差权重（可调）
        const float posWeight = 1.0f;          // 位置误差权重（可调）

        for (int iter = 0; iter < MaxIterations; iter++)
        {
            Pose current = ForwardKinematics.ComputePoseFromAngles(robot, angles);

            /* ---------------- 1. 计算任务空间误差 ---------------- */

            // 位置误差
            Vector3 posErr = target.position - current.position;

            // 姿态误差（轴角形式）
            Quaternion qErr = target.rotation * Quaternion.Inverse(current.rotation);
            qErr.ToAngleAxis(out float angleDeg, out Vector3 axis);

            Vector3 oriErr = Vector3.zero;
            if (angleDeg > 1e-3f && axis.sqrMagnitude > 1e-6f)
            {
                axis.Normalize();
                oriErr = angleDeg * Mathf.Deg2Rad * axis;
            }

            // 误差收敛判定（只要你需要的位置 / 姿态满足即可）
            if (posErr.sqrMagnitude <= PositionTolerance * PositionTolerance &&
                oriErr.sqrMagnitude <= 1e-6f)
            {
                break;
            }

            /* ---------------- 2. 构建 6 x n 数值雅可比 ---------------- */

            float[,] J = new float[6, n];
            Pose basePose = current;
            float deltaRad = JacobianDeltaDeg * Mathf.Deg2Rad;

            for (int j = 0; j < n; j++)
            {
                var anglesPerturbed = (float[])angles.Clone();
                anglesPerturbed[j] += JacobianDeltaDeg;

                Pose pPert = ForwardKinematics.ComputePoseFromAngles(robot, anglesPerturbed);

                // --- 位置偏导 ---
                Vector3 dp = (pPert.position - basePose.position) / deltaRad;
                J[0, j] = dp.x;
                J[1, j] = dp.y;
                J[2, j] = dp.z;

                // --- 姿态偏导（轴角） ---
                Quaternion dq = pPert.rotation * Quaternion.Inverse(basePose.rotation);
                dq.ToAngleAxis(out float dAngleDeg, out Vector3 dAxis);
                
                Vector3 dOri = Vector3.zero;
                if (dAngleDeg > 1e-3f && dAxis.sqrMagnitude > 1e-6f)
                {
                    dAxis.Normalize();
                    dOri = dAngleDeg * Mathf.Deg2Rad * dAxis / deltaRad;
                }

                J[3, j] = dOri.x;
                J[4, j] = dOri.y;
                J[5, j] = dOri.z;
            }

            /* ---------------- 3. Jacobian Transpose 更新 ---------------- */

            float[] dqJoint = new float[n];

            for (int j = 0; j < n; j++)
            {
                dqJoint[j] =
                    posWeight * (
                        J[0, j] * posErr.x +
                        J[1, j] * posErr.y +
                        J[2, j] * posErr.z
                    ) +
                    oriWeight * (
                        J[3, j] * oriErr.x +
                        J[4, j] * oriErr.y +
                        J[5, j] * oriErr.z
                    );
            }

            /* ---------------- 4. 步进、限幅、关节约束 ---------------- */

            for (int j = 0; j < n; j++)
            {
                float deltaDeg = JacobianStep * dqJoint[j] * Mathf.Rad2Deg;
                deltaDeg = Mathf.Clamp(deltaDeg, -JacobianStepLimitDeg, JacobianStepLimitDeg);

                angles[j] = Mathf.Clamp(
                    angles[j] + deltaDeg,
                    robot.RobotConfig.JointsParameters[j].AngleMin,
                    robot.RobotConfig.JointsParameters[j].AngleMax
                );
            }
        }

        return angles;
    }

    public void SwitchMethod()
    {
        // 切换逆向运动学方法
        IKMethod[] values = (IKMethod[])Enum.GetValues(typeof(IKMethod));
        int currentIndex = Array.IndexOf(values, Method);
        int nextIndex = (currentIndex + 1) % values.Length;
        Method = values[nextIndex];
    }
}
