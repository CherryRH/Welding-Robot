using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接规划器
/// </summary>
public class WeldPlanner
{
    public LinkedList<WeldInstruction> Instructions = new();

    private WeldIKSolver ikSolver = new();

    private Vector3 userCoordinateSystem;

    /// <summary>
    /// 焊接工作参数
    /// </summary>
    private float safetyHeight = 0.2f;
    private float moveSpeed = 0.4f;
    private float adjustSpeed = 0.2f;

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
            var firstSample = sampler.Samples[seam.ID][0];
            var lastSample = sampler.Samples[seam.ID][sampler.GetSampleCount(seam.ID)-1];

            if (!isContinous)
            {
                // 调整到安全高度
                Instructions.AddLast(
                    WeldInstruction.AdjustTo(
                        GetSafePose(pose),
                        adjustSpeed)
                );

                // 移动到焊缝的安全高度
                Instructions.AddLast(
                    WeldInstruction.MoveTo(
                        GetSafePose(firstSample),
                        moveSpeed,
                        continous: false,
                        pause: 1.0f)
                );

                // 从安全高度下降到焊缝起点
                Instructions.AddLast(
                    WeldInstruction.AdjustTo(
                        firstSample,
                        adjustSpeed,
                        pause: 1.0f)
                );
            }

            // 焊接
            Instructions.AddLast(
                WeldInstruction.WeldTo(
                    lastSample,
                    seam.Speed,
                    continous: seam.IsContinous)
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
    }

    public void PlanTrajectory(RobotModel robot, WeldSeamSampler sampler, Trajectory trajectory) {
        // 规划焊接任务轨迹

    }

    private Pose GetSafePose(Pose pose)
    {
        // 获取位姿的安全高度点
        Vector3 pos = pose.position;
        Vector3 safePos = new Vector3(pos.x, pos.y, userCoordinateSystem.z + safetyHeight);
        return new Pose(safePos, pose.rotation);
    }
}
