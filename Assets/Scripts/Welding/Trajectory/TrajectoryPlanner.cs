using System;
using System.Collections;
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

    public List<TrajectoryPlanResult> Plan(List<TcpPathPoint> points, float currentTime)
    {
        List<TrajectoryPlanResult> results = new();
        if (points == null || points.Count < 2) return results;
        for (int i = 0; i < points.Count-1; i++)
        {
            TcpPathPoint start = points[i];
            TcpPathPoint end = points[i+1];
            // 路径段结束
            if (start.Flag == TcpPathPoint.PointFlag.End)
            {
                results.Add(new TrajectoryPlanResult(start, end)
                {
                    PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Completed
                });
                continue;
            }
            switch (start.Type)
            {
                case TcpPathPoint.PointType.Approach:
                    results.Add(PlanApproachTrajectory(start, end, currentTime));
                    break;
                case TcpPathPoint.PointType.Weld:
                    results.Add(PlanWeldTrajectory(start, end, currentTime));
                    break;
                case TcpPathPoint.PointType.Adjust:
                    results.Add(PlanAdjustTrajectory(start, end, currentTime));
                    break;
            }
        }
        return results;
    }

    private TrajectoryPlanResult PlanApproachTrajectory(TcpPathPoint start, TcpPathPoint end, float currentTime)
    {
        // 规划移动任务轨迹
        TrajectoryPlanResult result = new(start, end);

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
        float endTime = startTime + length / robot.Config.TCPMaxSpeed;
        // 生成移动轨迹段
        // TODO: 要考虑前一段轨迹是否是连续的
        trajectory.Add(TrajectorySegment.BuildLinearSegment(
            TrajectorySegment.TrajectorySegmentType.Move,
            startTime, endTime,
            startPose, endPose,
            startJoints, endJoints));

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Completed;
        result.JointAngles = endJoints;
        return result;
    }

    private TrajectoryPlanResult PlanWeldTrajectory(TcpPathPoint start, TcpPathPoint end, float currentTime)
    {
        // 规划移动任务轨迹
        TrajectoryPlanResult result = new(start, end);

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
        float endTime = startTime + length / robot.Config.TCPMaxSpeed;
        // 生成移动轨迹段
        // TODO: 要考虑前一段轨迹是否是连续的
        trajectory.Add(TrajectorySegment.BuildLinearSegment(
            TrajectorySegment.TrajectorySegmentType.Move,
            startTime, endTime,
            startPose, endPose,
            startJoints, endJoints));

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Completed;
        result.JointAngles = endJoints;
        return result;
    }

    private TrajectoryPlanResult PlanAdjustTrajectory(TcpPathPoint start, TcpPathPoint end, float currentTime)
    {
        // 规划移动任务轨迹
        TrajectoryPlanResult result = new(start, end);

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

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Completed;
        result.JointAngles = endJoints;
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
        Completed,
        Unreachable,
        AdjustmentNeeded,
        Unknown
    }
    public TrajectoryPlanStatus PlanStatus = TrajectoryPlanStatus.Unknown;

    // 目标位姿对应的关节角度
    public float[] JointAngles;

    // 对应的路径点
    public TcpPathPoint StartPoint;
    public TcpPathPoint EndPoint;

    public TrajectoryPlanResult(TcpPathPoint startPoint, TcpPathPoint endPoint)
    {
        StartPoint = startPoint;
        EndPoint = endPoint;
    }
}