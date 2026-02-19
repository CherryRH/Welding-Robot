using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using Unity.VisualScripting;
using UnityEngine;

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
        TrajectoryPlanResult result = new();
        if (points == null || points.Count < 2)
        {
            return result;
        }
        var pathType = points[0].Type;

        switch (InterpolationMethod)
        {
            case InterpolationMethodType.Linear:
                result = PlanLinearTrajectory(points, currentTime);
                break;
            case InterpolationMethodType.CubicHermite:
                result = PlanCubicHermiteTrajectory(points, currentTime);
                break;
        }

        return result;
    }

    private TrajectoryPlanResult PlanLinearTrajectory(List<TcpPathPoint> points, float currentTime)
    {
        // 规划关节空间线性插值下的轨迹
        TrajectoryPlanResult result = new();

        // 记录每个点的时间和关节角
        List<float> timeList = new();
        List<float[]> jointsList = new();

        bool isTrajectoryEmpty = !trajectory.HasSegment;
        float time = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
        float[] joints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
        timeList.Add(time);
        jointsList.Add(joints);

        int n = points.Count - 1;
        for (int i = 0; i < n; i++)
        {
            var start = points[i];
            var end = points[i+1];
            // 确定起始关节角度
            float[] startJoints = joints;
            // 求解结束关节角度
            // TODO: 后续增加解的选择、冗余度处理和解算失败处理等
            float[] endJoints = robot.IK.Solve(end.Pose, startJoints) ?? startJoints;
            joints = endJoints;

            // 确定当前轨迹的起始时间和结束时间
            float startTime = time;
            float endTime = startTime + PlanTrajectoryDuration(start, end, startJoints, endJoints);
            time = endTime;

            timeList.Add(endTime);
            jointsList.Add(endJoints);
        }

        // 生成轨迹
        for (int i = 0; i < n; i++)
        {
            // 构造关节角插值器
            LinearJointInterpolator inter = new();
            inter.Build(jointsList[i+1], jointsList[i], timeList[i+1] - timeList[i]);

            // 生成移动轨迹段
            trajectory.Add(new TrajectorySegment(
                TrajectorySegment.TrajectorySegmentType.Move,
                timeList[i], timeList[i+1],
                jointsList[i], jointsList[i+1],
                inter));
        }

        return result;
    }

    private TrajectoryPlanResult PlanCubicHermiteTrajectory(List<TcpPathPoint> points, float currentTime)
    {
        // 规划关节空间三次Hermite样条插值下的轨迹
        TrajectoryPlanResult result = new();

        // 记录每个点的时间和关节角
        List<float> timeList = new();
        List<float[]> jointsList = new();

        bool isTrajectoryEmpty = !trajectory.HasSegment;
        float time = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
        float[] joints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
        timeList.Add(time);
        jointsList.Add(joints);

        int n = points.Count - 1;
        for (int i = 0; i < n; i++)
        {
            var start = points[i];
            var end = points[i + 1];
            // 确定起始关节角度
            float[] startJoints = joints;
            // 求解结束关节角度
            // TODO: 后续增加解的选择、冗余度处理和解算失败处理等
            float[] endJoints = robot.IK.Solve(end.Pose, startJoints) ?? startJoints;
            joints = endJoints;

            // 确定当前轨迹的起始时间和结束时间
            float startTime = time;
            float endTime = startTime + PlanTrajectoryDuration(start, end, startJoints, endJoints);
            time = endTime;

            timeList.Add(endTime);
            jointsList.Add(endJoints);
        }

        // 计算每个点的关节角速度（Fritsch–Carlson 单调限制）
        List<float[]> velocities = new();

        // 初始化
        for (int i = 0; i < n + 1; i++)
            velocities.Add(new float[dof]);

        for (int j = 0; j < dof; j++)
        {
            float[] h = new float[n];
            float[] delta = new float[n];
            float[] m = new float[n+1];
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
            for (int i = 0; i < n+1; i++)
            {
                velocities[i][j] = m[i];
                if (float.IsNaN(velocities[i][j]) || float.IsInfinity(velocities[i][j]))
                {
                    velocities[i][j] = 0f;
                }
            }
        }


        // 生成轨迹
        for (int i = 0; i < n; i++)
        {
            // 构造关节角插值器
            CubicHermiteSegmentInterpolator inter = new();
            inter.Build(jointsList[i], jointsList[i+1], velocities[i], velocities[i+1], timeList[i+1] - timeList[i]);

            // 生成移动轨迹段
            trajectory.Add(new TrajectorySegment(
                TrajectorySegment.TrajectorySegmentType.Move,
                timeList[i], timeList[i + 1],
                jointsList[i], jointsList[i + 1],
                inter));
        }

        return result;
    }

    private float PlanTrajectoryDuration(TcpPathPoint start, TcpPathPoint end, float[] startJoints, float[] endJoints)
    {
        // 规划轨迹所需的时间
        float duration = 0f;
        switch (start.Type)
        {
            case TcpPathPoint.PointType.Weld:
                duration = GetTcpReferenceDuration(start.Pose, end.Pose, start.Speed);
                break;
            case TcpPathPoint.PointType.Approach:
            case TcpPathPoint.PointType.Adjust:
            default:
                duration = Mathf.Max(
                    GetTcpReferenceDuration(start.Pose, end.Pose, start.Speed),
                    GetJointLimitedDuration(startJoints, endJoints));
                break;
        }
        return duration;
    }

    private float GetTcpReferenceDuration(Pose startPose, Pose endPose, float speed)
    {
        // 参考TCP线速度所需的时间
        float pathLength = Vector3.Distance(startPose.position, endPose.position);
        if (speed <= 1e-5f || pathLength <= 1e-6f)
            return 0f;
        return pathLength / speed;
    }

    private float GetJointLimitedDuration(float[] startJoints, float[] endJoints)
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
        return minDuration;
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

    // 对应的路径点
    public TcpPathPoint StartPoint;
    public TcpPathPoint EndPoint;
}