using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static TrajectorySegment;

public class TrajectoryPlanner
{
    public enum InterpolationMethodType
    {
        Linear,
        CubicHermite
    }
    public InterpolationMethodType InterpolationMethod = InterpolationMethodType.CubicHermite;

    private RobotModel robot;

    private Trajectory trajectory;

    private int dof;

    public void Init(RobotModel robot, Trajectory trajectory)
    {
        this.robot = robot;
        this.dof = robot.JointsCount;
        this.trajectory = trajectory;
    }

    public TrajectoryPlanResult Plan(List<TcpPathPoint> points, float currentTime)
    {
        // 规划关节空间的轨迹
        TrajectoryPlanResult result = new();
        if (points == null || points.Count < 2)
        {
            return result;
        }

        // 路径类型
        TrajectorySegmentType segmentType = points[0].Type switch
        {
            TcpPathPoint.PointType.Weld => TrajectorySegmentType.Weld,
            TcpPathPoint.PointType.Approach => TrajectorySegmentType.Approach,
            TcpPathPoint.PointType.Adjust => TrajectorySegmentType.Adjust,
            _ => TrajectorySegmentType.Approach
        };
        // 记录每个点的时间和关节角
        List<float> timeList = new();
        List<float[]> jointsList = new();

        bool isTrajectoryEmpty = !trajectory.HasActiveSegment;
        float time = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
        float[] joints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
        timeList.Add(time);
        jointsList.Add(joints);

        for (int i = 0; i < points.Count - 1; i++)
        {
            var start = points[i];
            var end = points[i + 1];
            // 确定起始关节角度
            float[] startJoints = joints;
            // 确定结束关节角度
            float[] endJoints = PlanEndJoints(startJoints, start, end);
            joints = endJoints;

            // 确定当前轨迹的起始时间和结束时间
            float startTime = time;
            float duration = PlanTrajectoryDuration(start, end, startJoints, endJoints);
            float endTime = startTime + duration;
            time = endTime;

            // 检查关节是否会超速
            if (robot.ViolatesJointVelocityLimit(startJoints, endJoints, duration))
            {
                result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.JointSpeedLimitViolated;
                result.CurrentPoint = start;
                break;
            }

            timeList.Add(endTime);
            jointsList.Add(endJoints);
            result.CurrentPoint = end;
        }

        if (timeList.Count > 1 && jointsList.Count > 1)
        {
            // 根据选择的插值方法构造轨迹段
            switch (InterpolationMethod)
            {
                case InterpolationMethodType.Linear:
                    BuildLinearTrajectory(points, timeList, jointsList, segmentType);
                    break;
                case InterpolationMethodType.CubicHermite:
                    List<float[]> velocities = PlanJointVelocities(timeList, jointsList);
                    BuildCubicHermiteTrajectory(points, timeList, jointsList, velocities, segmentType);
                    break;
            }
        }

        return result;
    }

    private void BuildLinearTrajectory(List<TcpPathPoint> points, List<float> timeList, List<float[]> jointsList, TrajectorySegmentType segmentType)
    {
        // 构造关节空间线性插值下的轨迹
        for (int i = 0; i < timeList.Count-1; i++)
        {
            // 构造关节角插值器
            LinearJointInterpolator inter = new();
            inter.Build(jointsList[i], jointsList[i+1], timeList[i + 1] - timeList[i]);

            // 生成移动轨迹段
            trajectory.Add(new TrajectorySegment(
                segmentType,
                points[i], points[i + 1],
                timeList[i], timeList[i + 1],
                jointsList[i], jointsList[i + 1],
                inter));
        }
    }

    private void BuildCubicHermiteTrajectory(List<TcpPathPoint> points, List<float> timeList, List<float[]> jointsList, List<float[]> velocities, TrajectorySegmentType segmentType)
    {
        // 构造关节空间三次Hermite样条插值下的轨迹段
        for (int i = 0; i < timeList.Count-1; i++)
        {
            // 构造关节角插值器
            CubicHermiteSegmentInterpolator inter = new();
            inter.Build(jointsList[i], jointsList[i + 1], velocities[i], velocities[i + 1], timeList[i + 1] - timeList[i]);

            // 生成移动轨迹段
            trajectory.Add(new TrajectorySegment(
                segmentType,
                points[i], points[i + 1],
                timeList[i], timeList[i + 1],
                jointsList[i], jointsList[i + 1],
                inter));
        }
    }

    private float[] PlanEndJoints(float[] startJoints, TcpPathPoint start, TcpPathPoint end)
    {
        // 规划结束关节角度
        switch (start.Flag)
        {
            case TcpPathPoint.PointFlag.SingularityApproach:
                {
                    // 进入奇异状态，J5归零
                    float[] singularity = (float[])startJoints.Clone();
                    singularity[4] = 0f;
                    return singularity;
                }
            case TcpPathPoint.PointFlag.FlipWrist:
                {
                    // 调整腕部姿态
                    float[] adjusted = (float[])startJoints.Clone();

                    // wrist flip
                    adjusted[3] += 180f;
                    adjusted[5] += 180f;

                    // 角度归一化
                    adjusted[3] = MathUtil.NormalizeEulerAngle(adjusted[3]);
                    adjusted[5] = MathUtil.NormalizeEulerAngle(adjusted[5]);

                    return adjusted;
                }
            case TcpPathPoint.PointFlag.SingularityLeave:
                {
                    // 通过IK求解
                    float[] endJoints = robot.IK.Solve(end.Pose, startJoints) ?? startJoints;
                    return endJoints;
                }
            default:
                {
                    // 其他类型的点，直接通过IK求解
                    float[] endJoints = robot.IK.Solve(end.Pose, startJoints) ?? startJoints;
                    return endJoints;
                }
        }
    }

    private float PlanTrajectoryDuration(TcpPathPoint start, TcpPathPoint end, float[] startJoints, float[] endJoints)
    {
        // 规划轨迹所需的时间
        float duration = 0f;
        duration = start.Type switch
        {
            TcpPathPoint.PointType.Weld => GetTcpReferenceDuration(start.Pose, end.Pose, start.Speed),
            TcpPathPoint.PointType.Approach => Mathf.Max(GetTcpReferenceDuration(start.Pose, end.Pose, start.Speed), GetJointLimitedDuration(startJoints, endJoints)),
            TcpPathPoint.PointType.Adjust => Mathf.Max(GetTcpReferenceDuration(start.Pose, end.Pose, start.Speed), GetJointLimitedDuration(startJoints, endJoints, 2.0f)),
            _ => Mathf.Max(GetTcpReferenceDuration(start.Pose, end.Pose, start.Speed), GetJointLimitedDuration(startJoints, endJoints))
        };
        return duration;
    }

    private List<float[]> PlanJointVelocities(List<float> timeList, List<float[]> jointsList)
    {
        // 规划每个点的关节角速度（Fritsch–Carlson 单调限制）
        List<float[]> velocities = new();
        int n = timeList.Count - 1;
        if (n < 1) return velocities;

        // 初始化
        for (int i = 0; i < n + 1; i++)
            velocities.Add(new float[dof]);

        for (int j = 0; j < dof; j++)
        {
            float[] h = new float[n];
            float[] delta = new float[n];
            float[] m = new float[n + 1];
            // Step 1: 计算 h 和 delta
            for (int i = 0; i < n; i++)
            {
                h[i] = Mathf.Max(1e-6f, timeList[i + 1] - timeList[i]);
                delta[i] = (jointsList[i + 1][j] - jointsList[i][j]) / h[i];
            }
            // Step 2: 初始斜率
            m[0] = delta[0];
            m[n] = delta[n - 1];
            for (int i = 1; i < n; i++)
            {
                m[i] = 0.5f * (delta[i - 1] + delta[i]);
            }
            // Step 3: 单调性修正
            for (int i = 1; i < n; i++)
            {
                if (delta[i - 1] * delta[i] <= 0f)
                {
                    m[i] = 0f;
                }
            }
            // Step 4: 过冲限制
            for (int i = 0; i < n; i++)
            {
                if (Mathf.Abs(delta[i]) < 1e-6f)
                {
                    m[i] = 0f;
                    m[i + 1] = 0f;
                    continue;
                }
                float alpha = m[i] / delta[i];
                float beta = m[i + 1] / delta[i];
                float s = alpha * alpha + beta * beta;
                if (s > 9f && s > 1e-6f)
                {
                    float tau = 3f / Mathf.Sqrt(s);
                    m[i] = tau * alpha * delta[i];
                    m[i + 1] = tau * beta * delta[i];
                }
            }
            // 写回 velocities
            for (int i = 0; i < n + 1; i++)
            {
                velocities[i][j] = m[i];
                if (float.IsNaN(velocities[i][j]) || float.IsInfinity(velocities[i][j]))
                {
                    velocities[i][j] = 0f;
                }
            }
        }
        return velocities;
    }

    private float GetTcpReferenceDuration(Pose startPose, Pose endPose, float speed)
    {
        // 参考TCP线速度所需的时间
        float pathLength = Vector3.Distance(startPose.position, endPose.position);
        if (speed <= 1e-5f || pathLength <= 1e-6f)
            return 0f;
        return pathLength / speed;
    }

    private float GetJointLimitedDuration(float[] startJoints, float[] endJoints, float multiple = 1.2f)
    {
        // 参考关节角速度限制所需的时间
        float minDuration = 0f;
        int n = Mathf.Min(startJoints.Length, endJoints.Length);
        for (int j = 0; j < n; j++)
        {
            float vmax = Mathf.Max(1e-4f, robot.Config.JointsParameters[j].AngleVMax);
            float duration = Mathf.Abs(endJoints[j] - startJoints[j]) / vmax;
            minDuration = Mathf.Max(minDuration, duration);
        }
        return minDuration * multiple;
    }
}

/// <summary>
/// 轨迹规划结果
/// </summary>
public class TrajectoryPlanResult
{
    // 规划状态
    public enum TrajectoryPlanStatus
    {
        Ok,
        JointSpeedLimitViolated,
        TcpPositionUnreachable,
        SingularOrFlipDetected,
        CollisionPredicted
    }
    public TrajectoryPlanStatus PlanStatus = TrajectoryPlanStatus.Ok;

    // 规划到的路径点
    public TcpPathPoint CurrentPoint;
}