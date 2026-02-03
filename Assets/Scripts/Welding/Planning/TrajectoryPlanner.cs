using System.Collections;
using System.Collections.Generic;
using System.Net;
using UnityEngine;

public class TrajectoryPlanner
{
    // 当前焊缝采样点索引
    private int currentSeamID = 0;
    private int currentSampleIndex = 0;

    private const int MaxSegmentsPerPlan = 10;

    public TrajectoryPlanResult Plan(RobotModel robot, WeldInstruction instr, WeldSeamSampler sampler, Trajectory trajectory, float currentTime)
    {
        if (instr == null)
            return new();

        switch (instr.Type)
        {
            case WeldInstruction.WeldInstructionType.Move:
                return PlanMoveInstruction(robot, instr, sampler, trajectory, currentTime);
            case WeldInstruction.WeldInstructionType.Adjust:
                return PlanAdjustInstruction(robot, instr, sampler, trajectory, currentTime);
            case WeldInstruction.WeldInstructionType.Weld:
                return PlanWeldInstruction(robot, instr, sampler, trajectory, currentTime);
            default:
                return new();
        }
    }

    private TrajectoryPlanResult PlanMoveInstruction(RobotModel robot, WeldInstruction instr, WeldSeamSampler sampler, Trajectory trajectory, float currentTime)
    {
        // 规划移动任务轨迹
        TrajectoryPlanResult result = new();

        // Trajectory是否为空
        bool isTrajectoryEmpty = !trajectory.HasSegment;
        // 确定起始位姿
        Pose startPose = isTrajectoryEmpty ? robot.TCPPose : trajectory.LastSegment.EndTCP;
        // 确定结束位姿
        Pose endPose = instr.TargetPose;
        // 生成直线路径
        LineGeometry line = new(
            startPose.position,
            endPose.position,
            Vector3.forward);
        // 确定当前规划的起始时间
        float startTime = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
        // 确定起始关节角度
        float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
        // 求解结束关节角度
        // TODO: 后续增加解的选择、冗余度处理和解算失败处理等
        float[] endJoints = robot.IK.Solve(instr.TargetPose, startJoints) ?? startJoints;
        // 计算轨迹结束时间
        float endTime = startTime + line.Length / robot.Config.TCPMaxSpeed;
        // 生成移动轨迹段
        // TODO: 要考虑前一段轨迹是否是连续的
        trajectory.Add(TrajectorySegment.BuildLinearSegment(
            TrajectorySegment.TrajectorySegmentType.Move,
            startTime, endTime,
            startPose, instr.TargetPose,
            startJoints, endJoints));
        // 如果有停顿，生成停止轨迹段
        if (instr.Pause > 0)
        {
            trajectory.Add(TrajectorySegment.BuildStaySegment(
                endTime, endTime + instr.Pause,
                instr.TargetPose,
                endJoints));
        }

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Completed;
        result.JointAngles = endJoints;
        return result;
    }

    private TrajectoryPlanResult PlanAdjustInstruction(RobotModel robot, WeldInstruction instr, WeldSeamSampler sampler, Trajectory trajectory, float currentTime)
    {
        // 规划调整任务轨迹
        TrajectoryPlanResult result = new();

        bool isTrajectoryEmpty = !trajectory.HasSegment;
        Pose startPose = isTrajectoryEmpty ? robot.TCPPose : trajectory.LastSegment.EndTCP;
        Pose endPose = instr.TargetPose;
        LineGeometry line = new(
            startPose.position,
            endPose.position,
            Vector3.forward);

        float startTime = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
        
        float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
        // 求解结束关节角度
        // TODO: 后续增加解的选择、冗余度处理和解算失败处理等，以及调姿时可能会涉及的特殊解
        float[] endJoints = robot.IK.Solve(instr.TargetPose, startJoints) ?? startJoints;
        float endTime = startTime + line.Length / robot.Config.TCPMaxSpeed;

        // 生成移动轨迹段
        // TODO: 要考虑前一段轨迹是否是连续的
        trajectory.Add(TrajectorySegment.BuildLinearSegment(
            TrajectorySegment.TrajectorySegmentType.Adjust,
            startTime, endTime,
            startPose, instr.TargetPose,
            startJoints, endJoints));
        if (instr.Pause > 0)
        {
            trajectory.Add(TrajectorySegment.BuildStaySegment(
                endTime, endTime + instr.Pause,
                instr.TargetPose,
                endJoints));
        }

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Completed;
        result.JointAngles = endJoints;
        return result;
    }

    private TrajectoryPlanResult PlanWeldInstruction(RobotModel robot, WeldInstruction instr, WeldSeamSampler sampler, Trajectory trajectory, float currentTime)
    {
        // 规划焊接任务轨迹
        TrajectoryPlanResult result = new();

        // 获取焊缝采样点
        if (currentSeamID != instr.SeamID)
        {
            // 从头开始规划该焊缝
            currentSampleIndex = 0;
            currentSeamID = instr.SeamID;
        }
        var samples = sampler.TryGetSamples(currentSeamID);
        if (samples == null || samples.Count < 2)
        {
            Debug.LogWarning($"焊缝ID {currentSeamID} 采样点不足，跳过该焊缝");
            result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Unknown;
            return result;
        }

        int i = 0;
        while (i < MaxSegmentsPerPlan)
        {
            bool isTrajectoryEmpty = !trajectory.HasSegment;
            float startTime = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
            // 如果当前焊缝已经规划完
            if (currentSampleIndex >= samples.Count - 1)
            {
                // 如果有停顿，生成停止轨迹段
                if (instr.Pause > 0)
                {
                    trajectory.Add(TrajectorySegment.BuildStaySegment(
                        startTime, startTime + instr.Pause,
                        instr.TargetPose,
                        result.JointAngles));
                }
                currentSampleIndex = 0;
                result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.Completed;
                return result;
            }

            Pose startPose = isTrajectoryEmpty ? robot.TCPPose : trajectory.LastSegment.EndTCP;
            Pose endPose = samples[currentSampleIndex + 1];
            // 将采样点转换到基座坐标系下
            endPose.position = robot.UserToRobot(endPose.position);

            float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;

            // IK（焊接阶段强烈建议：优先选与 startJoints 最近的解）
            float[] endJoints = robot.IK.Solve(endPose, startJoints) ?? startJoints;
            // 如果是最后一段，把关节角度存到结果中
            if (currentSampleIndex + 1 == samples.Count - 1)
            {
                result.JointAngles = endJoints;
            }

            LineGeometry line = new(
                startPose.position,
                endPose.position,
                Vector3.forward);
            float endTime = startTime + line.Length / instr.Speed;

            trajectory.Add(
                TrajectorySegment.BuildLinearSegment(
                    TrajectorySegment.TrajectorySegmentType.Weld,
                    startTime, endTime,
                    startPose, endPose,
                    startJoints, endJoints
                )
            );

            currentSampleIndex++;
            i++;
        }

        result.PlanStatus = TrajectoryPlanResult.TrajectoryPlanStatus.InProgress;
        return result;
    }

    public void Reset()
    {
        currentSeamID = 0;
        currentSampleIndex = 0;
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
        NotStarted,
        InProgress,
        Completed,
        Unreachable,
        AdjustmentNeeded,
        Unknown
    }
    public TrajectoryPlanStatus PlanStatus = TrajectoryPlanStatus.Unknown;

    // 目标位姿对应的关节角度
    public float[] JointAngles;
}