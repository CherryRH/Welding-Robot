using System.Collections.Generic;
using UnityEngine;

public class TrajectoryPlanner
{
    public int SegmentsPerPlan = 5;

    private RobotModel robot;
    private Trajectory trajectory;

    public void Init(RobotModel robot, Trajectory trajectory)
    {
        this.robot = robot;
        this.trajectory = trajectory;
    }

    public TrajectoryPlanResult Plan(List<TcpPathPoint> points, float currentTime)
    {
        TrajectoryPlanResult result = new();
        if (points == null || points.Count < 2)
        {
            result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Ok;
            return result;
        }

        var pathType = points[0].Type;
        return pathType switch
        {
            TcpPathPoint.PointType.Approach => PlanNonWeldTrajectory(points, currentTime, TrajectorySegment.TrajectorySegmentType.Move),
            TcpPathPoint.PointType.Adjust => PlanNonWeldTrajectory(points, currentTime, TrajectorySegment.TrajectorySegmentType.Adjust),
            TcpPathPoint.PointType.Weld => PlanWeldTrajectory(points, currentTime),
            _ => result
        };
    }

    private TrajectoryPlanResult PlanWeldTrajectory(List<TcpPathPoint> points, float currentTime)
    {
        return PlanPath(
            points,
            currentTime,
            TrajectorySegment.TrajectorySegmentType.Weld,
            strictTcpSpeed: true,
            mapJointLimitViolationToTcpUnreachable: true);
    }

    private TrajectoryPlanResult PlanNonWeldTrajectory(
        List<TcpPathPoint> points,
        float currentTime,
        TrajectorySegment.TrajectorySegmentType segType)
    {
        return PlanPath(
            points,
            currentTime,
            segType,
            strictTcpSpeed: false,
            mapJointLimitViolationToTcpUnreachable: false);
    }

    private TrajectoryPlanResult PlanPath(
        List<TcpPathPoint> points,
        float currentTime,
        TrajectorySegment.TrajectorySegmentType segmentType,
        bool strictTcpSpeed,
        bool mapJointLimitViolationToTcpUnreachable)
    {
        TrajectoryPlanResult result = new { PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Ok };

        for (int i = 0; i < points.Count - 1; i++)
        {
            var start = points[i];
            var end = points[i + 1];

            bool isTrajectoryEmpty = !trajectory.HasSegment;
            Pose startPose = start.Pose;
            Pose endPose = end.Pose;

            float startTime = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
            float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
            float[] endJoints = robot.IK.Solve(endPose, startJoints);

            if (endJoints == null || endJoints.Length != startJoints.Length)
            {
                return BuildFailedResult(
                    TrajectoryPlanResult.TrajectoryPlanStatus.TcpPositionUnreachable,
                    start,
                    end);
            }

            float segmentDuration;
            bool jointLimitViolated;

            if (strictTcpSpeed)
            {
                float pathLength = Vector3.Distance(startPose.position, endPose.position);
                float tcpSpeed = Mathf.Max(1e-5f, start.Speed);
                segmentDuration = pathLength / tcpSpeed;
                segmentDuration = Mathf.Max(1e-4f, segmentDuration);

                jointLimitViolated = ViolatesJointVelocityLimit(startJoints, endJoints, segmentDuration);
                if (jointLimitViolated)
                {
                    return BuildFailedResult(
                        mapJointLimitViolationToTcpUnreachable
                            ? TrajectoryPlanResult.TrajectoryPlanStatus.TcpSpeedUnreachable
                            : TrajectoryPlanResult.TrajectoryPlanStatus.JointSpeedLimitViolated,
                        start,
                        end);
                }
            }
            else
            {
                float tcpBasedDuration = GetTcpReferenceDuration(startPose, endPose, start.Speed);
                float jointLimitedDuration = GetJointLimitedDuration(startJoints, endJoints);
                segmentDuration = Mathf.Max(tcpBasedDuration, jointLimitedDuration, 1e-4f);

                jointLimitViolated = ViolatesJointVelocityLimit(startJoints, endJoints, segmentDuration);
                if (jointLimitViolated)
                {
                    return BuildFailedResult(
                        TrajectoryPlanResult.TrajectoryPlanStatus.JointSpeedLimitViolated,
                        start,
                        end);
                }
            }

            float endTime = startTime + segmentDuration;
            trajectory.Add(TrajectorySegment.BuildQuinticSegment(
                segmentType,
                startTime,
                endTime,
                startPose,
                endPose,
                startJoints,
                endJoints));
        }

        return result;
    }

    private float GetTcpReferenceDuration(Pose startPose, Pose endPose, float speed)
    {
        float pathLength = Vector3.Distance(startPose.position, endPose.position);
        if (speed <= 1e-5f || pathLength <= 1e-6f)
            return 0f;
        return pathLength / speed;
    }

    private float GetJointLimitedDuration(float[] startJoints, float[] endJoints)
    {
        float minDuration = 0f;
        int n = Mathf.Min(startJoints.Length, endJoints.Length);
        for (int j = 0; j < n; j++)
        {
            float vmax = Mathf.Max(1e-4f, robot.RobotConfig.JointsParameters[j].AngleVMax);
            float duration = Mathf.Abs(endJoints[j] - startJoints[j]) / vmax;
            if (duration > minDuration)
                minDuration = duration;
        }
        return minDuration;
    }

    private bool ViolatesJointVelocityLimit(float[] startJoints, float[] endJoints, float duration)
    {
        if (duration <= 1e-6f) return false;

        int n = Mathf.Min(startJoints.Length, endJoints.Length);
        for (int j = 0; j < n; j++)
        {
            float delta = Mathf.Abs(endJoints[j] - startJoints[j]);
            float requiredVelocity = delta / duration;
            float vmax = robot.RobotConfig.JointsParameters[j].AngleVMax;
            if (requiredVelocity > vmax + 1e-4f)
                return true;
        }
        return false;
    }

    private TrajectoryPlanResult BuildFailedResult(
        TrajectoryPlanResult.TrajectoryPlanStatus status,
        TcpPathPoint start,
        TcpPathPoint end)
    {
        return new TrajectoryPlanResult
        {
            PlanStatus = status,
            StartPoint = start,
            EndPoint = end
        };
    }
}

/// <summary>
/// 轨迹规划结果
/// </summary>
public class TrajectoryPlanResult
{
    public enum TrajectoryPlanStatus
    {
        Ok,
        JointSpeedLimitViolated,
        TcpSpeedUnreachable,
        TcpPositionUnreachable,
        SingularOrFlipDetected,
        CollisionPredicted,
        Unknown
    }

    public TrajectoryPlanStatus PlanStatus = TrajectoryPlanStatus.Unknown;

    public TcpPathPoint StartPoint;
    public TcpPathPoint EndPoint;
}
