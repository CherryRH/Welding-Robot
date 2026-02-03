using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接规划器，负责指令序列规划与重规划
/// </summary>
public class WeldPlanner
{
    public LinkedList<WeldInstruction> Instructions = new();

    private LinkedListNode<WeldInstruction> currentInstruction;

    /// <summary>
    /// 焊接工作参数
    /// </summary>
    private const float moveSpeed = 0.2f;
    private const float adjustSpeed = 0.2f;
    private const float pauseDuration = 1.0f;

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
            firstSample.position = robot.UserToRobot(firstSample.position);
            lastSample.position = robot.UserToRobot(lastSample.position);

            if (!isContinous)
            {
                // 调整到安全高度，姿态先与焊缝起点保持一致
                Instructions.AddLast(
                    WeldInstruction.AdjustTo(
                        robot.GetSafePose(new(pose.position, firstSample.rotation)),
                        adjustSpeed)
                );

                // 移动到焊缝的安全高度
                Instructions.AddLast(
                    WeldInstruction.MoveTo(
                        robot.GetSafePose(firstSample),
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
                robot.GetSafePose(pose),
                adjustSpeed)
        );

        // 当前指令设为起始指令的后一个
        currentInstruction = Instructions.First.Next;
    }

    public WeldInstruction GetInstruction()
    {
        if (currentInstruction == null)
            return null;
        WeldInstruction instr = currentInstruction.Value;
        return instr;
    }

    public void HandleTrajectoryPlanResult(TrajectoryPlanResult result)
    {
        switch (result.PlanStatus)
        {
            case TrajectoryPlanResult.TrajectoryPlanStatus.NotStarted:
                break;
            case TrajectoryPlanResult.TrajectoryPlanStatus.InProgress:
                break;
            case TrajectoryPlanResult.TrajectoryPlanStatus.Completed:
                // 保存仿真结果，移动到下一指令
                WeldInstruction instr = currentInstruction.Value;
                instr.BindJoints(result.JointAngles);
                currentInstruction = currentInstruction.Next;
                break;
            case TrajectoryPlanResult.TrajectoryPlanStatus.Unreachable:
                break;
            case TrajectoryPlanResult.TrajectoryPlanStatus.AdjustmentNeeded:
                break;
            case TrajectoryPlanResult.TrajectoryPlanStatus.Unknown:
                break;
            default:
                break;
        }
    }

    public void Reset()
    {
        Instructions.Clear();
        currentInstruction = null;
    }
}
