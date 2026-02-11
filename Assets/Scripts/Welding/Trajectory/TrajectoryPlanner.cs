using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
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
        if (points == null || points.Count < 2) return result;
        var pathType = points[0].Type;
        switch (pathType)
        {
            case TcpPathPoint.PointType.Approach:
                result = PlanApproachTrajectory(points, currentTime);
                break;
            case TcpPathPoint.PointType.Weld:
                result = PlanWeldTrajectory(points, currentTime);
                break;
            case TcpPathPoint.PointType.Adjust:
                result = PlanAdjustTrajectory(points, currentTime);
                break;
        }
        return result;
    }

    private TrajectoryPlanResult PlanApproachTrajectory(List<TcpPathPoint> points, float currentTime)
    {
        // 规划移动任务轨迹
        TrajectoryPlanResult result = new();
        for (int i = 0; i < points.Count-1; i++)
        {
            var start = points[i];
            var end = points[i+1];
            // Trajectory是否为空
            bool isTrajectoryEmpty = !trajectory.HasSegment;
            // 起始位姿
            Pose startPose = start.Pose;
            // 结束位姿
            Pose endPose = end.Pose;
            // 计算直线路径长度
            float length = Vector3.Distance(startPose.position, endPose.position);
            // 确定当前规划的起始时间
            float startTime = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
            // 确定起始关节角度
            float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
            // 求解结束关节角度
            // TODO: 后续增加解的选择、冗余度处理和解算失败处理等
            float[] endJoints = robot.IK.Solve(endPose, startJoints) ?? startJoints;
            // 计算轨迹结束时间
            float endTime = startTime + length / start.Speed;
            // 生成移动轨迹段
            // TODO: 要考虑前一段轨迹是否是连续的
            trajectory.Add(TrajectorySegment.BuildLinearSegment(
                TrajectorySegment.TrajectorySegmentType.Move,
                startTime, endTime,
                startPose, endPose,
                startJoints, endJoints));
        }

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Ok;
        return result;
    }

    private TrajectoryPlanResult PlanWeldTrajectory(List<TcpPathPoint> points, float currentTime)
    {
        // 规划移动任务轨迹
        TrajectoryPlanResult result = new();
        for (int i = 0; i < points.Count - 1; i++)
        {
            var start = points[i];
            var end = points[i + 1];
            // Trajectory是否为空
            bool isTrajectoryEmpty = !trajectory.HasSegment;
            // 起始位姿
            Pose startPose = start.Pose;
            // 结束位姿
            Pose endPose = end.Pose;
            // 计算直线路径长度
            float length = Vector3.Distance(startPose.position, endPose.position);
            // 确定当前规划的起始时间
            float startTime = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
            // 确定起始关节角度
            float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
            // 求解结束关节角度
            // TODO: 后续增加解的选择、冗余度处理和解算失败处理等
            float[] endJoints = robot.IK.Solve(endPose, startJoints) ?? startJoints;
            // 计算轨迹结束时间
            float endTime = startTime + length / start.Speed;
            // 生成移动轨迹段
            // TODO: 要考虑前一段轨迹是否是连续的
            trajectory.Add(TrajectorySegment.BuildLinearSegment(
                TrajectorySegment.TrajectorySegmentType.Move,
                startTime, endTime,
                startPose, endPose,
                startJoints, endJoints));
        }

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Ok;
        return result;
    }

    private TrajectoryPlanResult PlanAdjustTrajectory(List<TcpPathPoint> points, float currentTime)
    {
        // 规划移动任务轨迹
        TrajectoryPlanResult result = new();
        for (int i = 0; i < points.Count - 1; i++)
        {
            var start = points[i];
            var end = points[i + 1];
            // Trajectory是否为空
            bool isTrajectoryEmpty = !trajectory.HasSegment;
            // 起始位姿
            Pose startPose = start.Pose;
            // 结束位姿
            Pose endPose = end.Pose;
            // 计算直线路径长度
            float length = Vector3.Distance(startPose.position, endPose.position);
            // 确定当前规划的起始时间
            float startTime = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
            // 确定起始关节角度
            float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
            // 求解结束关节角度
            // TODO: 后续增加解的选择、冗余度处理和解算失败处理等
            float[] endJoints = robot.IK.Solve(endPose, startJoints) ?? startJoints;
            // 计算轨迹结束时间
            float endTime = startTime + length / start.Speed;
            // 生成移动轨迹段
            // TODO: 要考虑前一段轨迹是否是连续的
            trajectory.Add(TrajectorySegment.BuildLinearSegment(
                TrajectorySegment.TrajectorySegmentType.Move,
                startTime, endTime,
                startPose, endPose,
                startJoints, endJoints));
        }

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Ok;
        return result;
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
        TcpSpeedUnreachable,
        TcpPositionUnreachable,
        SingularOrFlipDetected,
        CollisionPredicted,
        Unknown
    }
    public TrajectoryPlanStatus PlanStatus = TrajectoryPlanStatus.Unknown;

    // 对应的路径点
    public TcpPathPoint StartPoint;
    public TcpPathPoint EndPoint;
}