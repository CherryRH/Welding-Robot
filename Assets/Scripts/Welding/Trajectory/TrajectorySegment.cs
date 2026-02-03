using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 路径片段
/// </summary>
public class TrajectorySegment
{
    public enum TrajectorySegmentType
    {
        Move,
        Weld,
        Adjust,
        Stay
    }
    public TrajectorySegmentType Type;

    public float StartTime;
    public float EndTime;

    public Pose StartTCP;
    public Pose EndTCP;

    public float[] QStart;
    public float[] QEnd;

    public Interpolation[] Interpolations;

    public float[] Evaluate(float simTime)
    {
        // 计算 simTime 时刻的关节位置
        float t = Mathf.Clamp(simTime - StartTime, 0f, EndTime - StartTime);
        int n = Interpolations.Length;
        float[] qPos = new float[n];
        for (int i = 0; i < n; i++)
        {
            qPos[i] = Interpolations[i].Evaluate(t);
        }
        return qPos;
    }

    public static TrajectorySegment BuildStaySegment(float startTime, float endTime, Pose tcpPose, float[] qPose)
    {
        // 创建停留的路径片段
        TrajectorySegment segment = new();
        segment.Type = TrajectorySegmentType.Stay;
        segment.StartTime = startTime;
        segment.EndTime = endTime;
        segment.StartTCP = tcpPose;
        segment.EndTCP = tcpPose;
        segment.QStart = qPose;
        segment.QEnd = qPose;
        int n = qPose.Length;
        segment.Interpolations = new Interpolation[n];
        for (int i = 0; i < n; i++)
        {
            LinearInterpolation interpolation = new();
            interpolation.Build(qPose[i], qPose[i], endTime - startTime);
            segment.Interpolations[i] = interpolation;
        }
        return segment;
    }

    public static TrajectorySegment BuildLinearSegment(TrajectorySegmentType type, float startTime, float endTime, Pose startTCP, Pose endTCP, float[] qStart, float[] qEnd)
    {
        // 创建线性插值的路径片段
        TrajectorySegment segment = new();
        segment.Type = type;
        segment.StartTime = startTime;
        segment.EndTime = endTime;
        segment.StartTCP = startTCP;
        segment.EndTCP = endTCP;
        segment.QStart = qStart;
        segment.QEnd = qEnd;
        int n = Mathf.Max(qStart.Length, qEnd.Length);
        segment.Interpolations = new Interpolation[n];
        for (int i = 0; i < n; i++)
        {
            LinearInterpolation interpolation = new();
            interpolation.Build(qStart[i], qEnd[i], endTime - startTime);
            segment.Interpolations[i] = interpolation;
        }
        return segment;
    }
}
