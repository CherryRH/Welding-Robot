using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 逆向运动学
/// </summary>
public class IK
{
    public enum IKMethod
    {
        JT,// Jacobian Transpose
        CCD,// Cyclic Coordinate Descent
        ANALYTIC // Anlytic
    }

    public IKMethod Method = IKMethod.ANALYTIC;

    // 公共参数，可按需调整
    public int MaxIterations = 100; // 数值法最大迭代次数
    public float PositionTolerance = 1e-3f; // 单位：米（与 TCP Position 单位一致）
    public float CCDStepLimitDeg = 10f; // CCD 单次关节调整限制（度）
    public float JacobianStep = 1.0f; // Jacobian Transpose 步长系数（调小以保证收敛）
    public float JacobianDeltaDeg = 1e-1f; // 数值雅可比扰动（度）
    public float JacobianStepLimitDeg = 5f; // Jacobian 单次关节调整限制（度）

    private RobotModel robot;

    public IK(RobotModel robotModel)
    {
        robot = robotModel;
    }

    public float[] Solve(Pose tcpPose, float[] current)
    {
        if (robot == null || robot.RobotConfig == null || robot.RobotConfig.JointsParameters == null)
            return new float[robot?.Joints.Length ?? 0];

        switch (Method)
        {
            case IKMethod.CCD:
                return SolveCCD(tcpPose).GetBestSolution(current);
            case IKMethod.JT:
                return SolveJacobianTranspose(tcpPose).GetBestSolution(current);
            case IKMethod.ANALYTIC:
                return SolveAnalytic(tcpPose).GetBestSolution(current);
            default:
                return SolveCCD(tcpPose).GetBestSolution(current);
        }
    }

    /// <summary>
    /// CCD（位置控制）
    /// </summary>
    private IKResult SolveCCD(Pose tcpPose)
    {
        IKResult result = new();

        int n = robot.Joints.Length;
        var angles = new float[n];
        for (int i = 0; i < n; i++) angles[i] = robot.Joints[i].Angle;

        for (int iter = 0; iter < MaxIterations; iter++)
        {
            // 计算当前 TCP
            Pose current = FK.ComputePoseFromAngles(robot, angles);
            Vector3 tcpPos = current.position;
            if ((tcpPos - tcpPose.position).sqrMagnitude <= PositionTolerance * PositionTolerance)
                break;

            // 从末端到基座逐关节调整
            for (int i = n - 1; i >= 0; i--)
            {
                // 计算第 i 个关节的世界位置（origin of joint frame）
                Vector3 jointPos = FK.ComputeJointWorldPosition(robot, angles, i);

                Vector3 toCurr = tcpPos - jointPos;
                Vector3 toTarget = tcpPose.position - jointPos;

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
                Vector3 jointZ = FK.ComputeJointZDirection(robot, angles, i);
                float sign = Mathf.Sign(Vector3.Dot(jointZ, cross));
                if (sign == 0) sign = 1f;

                float deltaDeg = Mathf.Clamp(0.5f * angleBetweenDeg * sign, -CCDStepLimitDeg, CCDStepLimitDeg); // 加入角度缩放

                angles[i] = Mathf.Clamp(angles[i] + deltaDeg,
                    robot.RobotConfig.JointsParameters[i].AngleMin,
                    robot.RobotConfig.JointsParameters[i].AngleMax);

                // 更新当前 TCP（局部更新即可）
                current = FK.ComputePoseFromAngles(robot, angles);
                tcpPos = current.position;

                if ((tcpPos - tcpPose.position).sqrMagnitude <= PositionTolerance * PositionTolerance)
                    break;
            }
        }

        result.Solutions.Add(angles);
        return result;
    }

    /// <summary>
    /// Jacobian Transpose数值雅可比（位置控制）
    /// </summary>
    private IKResult SolveJacobianTranspose(Pose tcpPose)
    {
        IKResult result = new();

        int n = robot.Joints.Length;
        var angles = new float[n];
        for (int i = 0; i < n; i++)
            angles[i] = robot.Joints[i].Angle;

        const float oriWeight = 0f;          // 姿态误差权重（可调）
        const float posWeight = 1.0f;          // 位置误差权重（可调）

        for (int iter = 0; iter < MaxIterations; iter++)
        {
            Pose current = FK.ComputePoseFromAngles(robot, angles);

            /* ---------------- 1. 计算任务空间误差 ---------------- */

            // 位置误差
            Vector3 posErr = tcpPose.position - current.position;

            // 姿态误差（轴角形式）
            Quaternion qErr = tcpPose.rotation * Quaternion.Inverse(current.rotation);
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

                Pose pPert = FK.ComputePoseFromAngles(robot, anglesPerturbed);

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

        result.Solutions.Add(angles);
        return result;
    }

    /// <summary>
    /// 解析法（位置、姿态控制）
    /// </summary>
    public IKResult SolveAnalytic(Pose tcpPose)
    {
        IKResult result = new();
        RobotConfig config = robot.RobotConfig;

        float wristOffset = config.FlangeOffset.z + config.TCPOffset.z;
        Vector3 wristPos = tcpPose.position - tcpPose.rotation * Vector3.forward * wristOffset;

        var j123List = SolveAnalyticWristPos(config, wristPos);
        if (j123List == null || j123List.Count == 0) return result;

        Matrix4x4 R06 = Matrix4x4.Rotate(tcpPose.rotation);

        foreach (var j123 in j123List)
        {
            Matrix4x4 R03 = FK.ComputeJointTransformMatrix(robot, j123, 2);
            Matrix4x4 R36 = R03.transpose * R06;

            float c5 = Mathf.Clamp(R36.m12, -1f, 1f);
            float t5 = Mathf.Acos(c5);

            float[] t5Candidates = { t5, -t5 };

            foreach (float theta5 in t5Candidates)
            {
                float theta4, theta6;
                float s5 = Mathf.Sin(theta5);

                if (Mathf.Abs(s5) < 1e-6f)
                {
                    // Z-Y-Z 奇异：θ4 + θ6 可辨，单独不可
                    theta4 = robot.JointAngles[3] * Mathf.Deg2Rad;
                    theta6 = Mathf.Atan2(R36.m20, R36.m00) - theta4;
                }
                else
                {
                    theta4 = Mathf.Atan2(R36.m22 / s5, -R36.m02 / s5);
                    theta6 = Mathf.Atan2(-R36.m11 / s5, R36.m10 / s5);
                }

                result.Solutions.Add(new float[]
                {
                    j123[0],
                    j123[1],
                    j123[2],
                    theta4 * Mathf.Rad2Deg,
                    theta5 * Mathf.Rad2Deg,
                    theta6 * Mathf.Rad2Deg
                });
            }
        }

        return result;
    }

    private List<float[]> SolveAnalyticWristPos(RobotConfig config, Vector3 wristPos)
    {
        List<float[]> solutions = new();

        // 获取 MDH 参数
        float d1 = config.JointsParameters[0].D; // J1 轴高
        float a1 = config.JointsParameters[1].A; // 肩部偏置 (L2前的偏置)
        float a2 = config.JointsParameters[2].A; // 大臂长度

        // 注意：在 MDH 中，J4 的 a3 和 d4 决定了从 J3 到腕心的位置
        float a3 = config.JointsParameters[3].A;
        float d4 = config.JointsParameters[3].D;

        // 1. 求解 J1 (θ1)
        // 根据右手系，θ1 = atan2(y, x)
        float theta1 = Mathf.Atan2(wristPos.y, wristPos.x);
        float[] theta1Candidates = { theta1, theta1 + Mathf.PI }; // 考虑正反两个方向

        foreach (float t1 in theta1Candidates)
        {
            // 将腕部中心投影到 J2-J3 转动平面上 (R-Z 平面)
            float r = Mathf.Sqrt(wristPos.x * wristPos.x + wristPos.y * wristPos.y);

            // 实际参与解析几何的 X 和 Y (相对于 J2 的原点，Y向前，X向上)
            float targetX = wristPos.z - d1;
            float targetY = r - a1;

            // 计算 J3 到腕心的等效长度 L3_eff 和 偏角 phi
            // 腕心在 Frame 3 中的坐标通常为 (a3, 0, d4) 或类似，取决于具体的 MDH 定义
            // 基于 M10iD 这种弯臂机器人：
            float L3_eff = Mathf.Sqrt(a3 * a3 + d4 * d4);
            float phi = Mathf.Atan2(d4, a3); // 腕心偏角

            // 2. 求解 J3 (θ3) 
            // 使用余弦定理: D^2 = a2^2 + L3_eff^2 - 2*a2*L3_eff*cos(beta)
            float distSq = targetX * targetX + targetY * targetY;
            float dist = Mathf.Sqrt(distSq);

            float cosBeta = (a2 * a2 + L3_eff * L3_eff - distSq) / (2 * a2 * L3_eff);

            // 检查是否在工作可达范围内
            if (cosBeta < -1f || cosBeta > 1f) continue;

            float beta = Mathf.Acos(cosBeta);

            // J3 有两个解（肘向上/肘向下）
            float[] t3Solutions = {
                -Mathf.PI + beta + phi,
                Mathf.PI - beta + phi
            };

            foreach (float t3 in t3Solutions)
            {
                // 3. 求解 J2 (θ2)
                // 使用几何关系：alpha 是坐标点与水平线的夹角，gamma 是 a2 与连线的夹角
                float alpha = Mathf.Atan2(targetY, targetX);
                float cosGamma = (a2 * a2 + distSq - L3_eff * L3_eff) / (2 * a2 * dist);
                if (cosGamma < -1f || cosGamma > 1f) continue;
                float gamma = Mathf.Acos(cosGamma);

                float t2;
                if (t3 > phi) // 肘向下
                    t2 = alpha + gamma;
                else // 肘向上
                    t2 = alpha - gamma;

                solutions.Add(new float[]
                {
                    t1 * Mathf.Rad2Deg,
                    t2 * Mathf.Rad2Deg,
                    -t3 * Mathf.Rad2Deg
                });
            }
        }
        return solutions;
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
