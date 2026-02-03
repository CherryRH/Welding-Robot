using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static WeldInstruction;

/// <summary>
/// 焊接规划器
/// </summary>
public class WeldPlanner
{
    public LinkedList<WeldInstruction> Instructions = new();

    private LinkedListNode<WeldInstruction> currentInstruction;

    // 当前焊缝采样点索引
    private int currentSeamID = 0;
    private int currentSampleIndex = 0;

    private Vector3 userCoordinateSystem;

    /// <summary>
    /// 焊接工作参数
    /// </summary>
    private const float safetyHeight = 0.2f;
    private const float moveSpeed = 0.2f;
    private const float adjustSpeed = 0.2f;
    private const float pauseDuration = 1.0f;

    /// <summary>
    /// 路径规划参数
    /// </summary>
    private const int maxSegmentsPerPlan = 10;

    public void Init(Vector3 ucs) {
        // 初始化用户坐标系为工件原点位置
        userCoordinateSystem = ucs;
    }

    public void PlanInstruction(RobotModel robot, WeldSeamSampler sampler, WeldTask task) {
        // 规划焊接任务指令序列
        Instructions.Clear();

        // 起始位置
        var start = WeldInstruction.MoveTo(
            robot.TCPPose,
            moveSpeed,
            continous: true);

        start.BindJoints(robot.JointAngles);
        Instructions.AddLast(start);
        
        // 当前焊缝是否是连续的
        bool isContinous = false;
        // 当前位姿
        Pose pose = robot.TCPPose;

        // 遍历焊缝
        foreach (var seam in task.WeldSeams)
        {
            var firstSample = sampler.TryGetSamples(seam.ID)[0];
            var lastSample = sampler.TryGetSamples(seam.ID)[sampler.GetSampleCount(seam.ID)-1];
            if (firstSample == null || lastSample == null)
            {
                Debug.LogWarning($"焊缝ID {seam.ID} 采样失败，跳过该焊缝");
                continue;
            }
            // 将采样点转换到基座坐标系下
            firstSample.position = firstSample.position + userCoordinateSystem;
            lastSample.position = lastSample.position + userCoordinateSystem;

            if (!isContinous)
            {
                // 调整到安全高度，姿态先与焊缝起点保持一致
                Instructions.AddLast(
                    WeldInstruction.AdjustTo(
                        GetSafePose(new(pose.position, firstSample.rotation)),
                        adjustSpeed)
                );

                // 移动到焊缝的安全高度
                Instructions.AddLast(
                    WeldInstruction.MoveTo(
                        GetSafePose(firstSample),
                        moveSpeed,
                        continous: false,
                        pause: pauseDuration)
                );

                // 从安全高度下降到焊缝起点
                Instructions.AddLast(
                    WeldInstruction.AdjustTo(
                        firstSample,
                        adjustSpeed,
                        pause: pauseDuration)
                );
            }

            // 焊接指令，绑定焊缝ID
            Instructions.AddLast(
                WeldInstruction.WeldTo(
                    lastSample,
                    seam.Speed,
                    seamID: seam.ID,
                    continous: seam.IsContinous,
                    pause: seam.IsContinous ? 0f : pauseDuration)
            );

            isContinous = seam.IsContinous;
            pose = lastSample;
        }

        // 最后调整到安全高度
        Instructions.AddLast(
            WeldInstruction.AdjustTo(
                GetSafePose(pose),
                adjustSpeed)
        );

        // 当前指令设为起始指令的后一个
        currentInstruction = Instructions.First.Next;
        currentSeamID = 0;
        currentSampleIndex = 0;
    }

    public void PlanTrajectory(RobotModel robot, WeldSeamSampler sampler, Trajectory trajectory, float currentTime) {
        if (currentInstruction == null) return;
        // 规划焊接任务轨迹
        // 取当前指令，每次最多只规划一条指令
        WeldInstruction instr = currentInstruction.Value;

        switch (instr.Type)
        {
            case WeldInstructionType.Move:
                {
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
                    // 计算轨迹结束时间
                    float endTime = startTime + line.Length / moveSpeed;
                    // 确定起始关节角度
                    float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
                    // 求解结束关节角度
                    // TODO: 后续增加解的选择、冗余度处理和解算失败处理等
                    float[] endJoints = robot.IK.Solve(instr.TargetPose, startJoints) ?? startJoints;
                    instr.BindJoints(endJoints);
                    // 生成移动轨迹段
                    // TODO: 要考虑前一段轨迹是否是连续的
                    trajectory.Add(TrajectorySegment.BuildLinearSegment(
                        TrajectorySegment.TrajectorySegmentType.Move,
                        startTime, endTime,
                        startPose, instr.TargetPose,
                        startJoints, instr.TargetJoints));
                    // 如果有停顿，生成停止轨迹段
                    if (instr.Pause > 0)
                    {
                        trajectory.Add(TrajectorySegment.BuildStaySegment(
                            endTime, endTime + instr.Pause,
                            instr.TargetPose,
                            instr.TargetJoints));
                    }
                    // 更新当前指令到下一条
                    currentInstruction = currentInstruction.Next;
                }
                break;
            case WeldInstructionType.Adjust:
                {
                    bool isTrajectoryEmpty = !trajectory.HasSegment;
                    Pose startPose = isTrajectoryEmpty ? robot.TCPPose : trajectory.LastSegment.EndTCP;
                    Pose endPose = instr.TargetPose;
                    LineGeometry line = new(
                        startPose.position,
                        endPose.position,
                        Vector3.forward);
                    float startTime = isTrajectoryEmpty ? currentTime : trajectory.LastSegment.EndTime;
                    float endTime = startTime + line.Length / adjustSpeed;
                    float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;
                    // 求解结束关节角度
                    // TODO: 后续增加解的选择、冗余度处理和解算失败处理等，以及调姿时可能会涉及的特殊解
                    float[] endJoints = robot.IK.Solve(instr.TargetPose, startJoints) ?? startJoints;
                    instr.BindJoints(endJoints);
                    // 生成移动轨迹段
                    // TODO: 要考虑前一段轨迹是否是连续的
                    trajectory.Add(TrajectorySegment.BuildLinearSegment(
                        TrajectorySegment.TrajectorySegmentType.Adjust,
                        startTime, endTime,
                        startPose, instr.TargetPose,
                        startJoints, instr.TargetJoints));
                    if (instr.Pause > 0)
                    {
                        trajectory.Add(TrajectorySegment.BuildStaySegment(
                            endTime, endTime + instr.Pause,
                            instr.TargetPose,
                            instr.TargetJoints));
                    }
                    currentInstruction = currentInstruction.Next;
                }
                break;
            case WeldInstructionType.Weld:
                {
                    currentSeamID = instr.SeamID;
                    var samples = sampler.TryGetSamples(currentSeamID);
                    if (samples == null || samples.Count < 2)
                    {
                        Debug.LogWarning($"焊缝ID {currentSeamID} 采样点不足，跳过该焊缝");
                        currentInstruction = currentInstruction.Next;
                        break;
                    }

                    int i = 0;
                    while (i < maxSegmentsPerPlan)
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
                                    instr.TargetJoints));
                            }
                            currentInstruction = currentInstruction.Next;
                            currentSampleIndex = 0;
                            break;
                        }

                        Pose startPose = isTrajectoryEmpty ? robot.TCPPose : trajectory.LastSegment.EndTCP;
                        Pose endPose = samples[currentSampleIndex + 1];
                        // 将采样点转换到基座坐标系下
                        endPose.position = endPose.position + userCoordinateSystem;

                        float[] startJoints = isTrajectoryEmpty ? robot.JointAngles : trajectory.LastSegment.QEnd;

                        // IK（焊接阶段强烈建议：优先选与 startJoints 最近的解）
                        float[] endJoints = robot.IK.Solve(endPose, startJoints) ?? startJoints;
                        // 如果是最后一段，绑定关节角度到指令
                        if (currentSampleIndex + 1 == samples.Count - 1)
                        {
                            instr.BindJoints(endJoints);
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
                }
                break;
        }
    }

    private Pose GetSafePose(Pose pose)
    {
        // 获取位姿的安全高度点
        Vector3 pos = pose.position;
        Vector3 safePos = new Vector3(pos.x, pos.y, userCoordinateSystem.z + safetyHeight);
        return new Pose(safePos, pose.rotation);
    }

    public void Reset()
    {
        Instructions.Clear();
        currentInstruction = null;
        currentSeamID = 0;
        currentSampleIndex = 0;
    }
}
