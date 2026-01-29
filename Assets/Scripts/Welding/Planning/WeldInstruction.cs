using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接任务指令
/// </summary>
[System.Serializable]
public class WeldInstruction
{
    // 指令类型
    public enum WeldInstructionType
    {
        Move,
        Weld,
        Adjust
    }
    public WeldInstructionType Type;

    // 目标位姿（开始时设置）
    public Pose TargetPose;

    // 目标关节角度（完成时设置）
    public float[] TargetJoints;

    // TCP速度（米/秒）
    public float Speed;

    // 与下一个指令是否连续
    public bool IsContinous;

    // 到达点位后的暂停时间（秒）
    public float PauseDuration;

    public WeldInstruction(WeldInstructionType type, Pose pose, float speed, bool isContinous, float pauseDuration)
    {
        Type = type;
        TargetPose = pose;
        Speed = speed;
        IsContinous = isContinous;
        PauseDuration = pauseDuration;
    }

    public void BindJoints(float[] joints)
    {
        TargetJoints = joints;
    }

    public static WeldInstruction MoveTo(Pose pose, float speed, bool continous, float pause = 0f)
    {
        return new WeldInstruction(WeldInstructionType.Move, pose, speed, continous, pause);
    }

    public static WeldInstruction WeldTo(Pose pose, float speed, bool continous, float pause = 0f)
    {
        return new WeldInstruction(WeldInstructionType.Weld, pose, speed, continous, pause);
    }

    public static WeldInstruction AdjustTo(Pose pose, float speed, float pause = 0f)
    {
        return new WeldInstruction(WeldInstructionType.Adjust, pose, speed, false, pause);
    }

}
