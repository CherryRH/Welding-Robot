using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 轨迹片段
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

    public IInterpolation[] Interpolations;

    public float[] Evaluate(float simTime)
    {
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
        TrajectorySegment segment = new();
        segment.Type = TrajectorySegmentType.Stay;
        segment.StartTime = startTime;
        segment.EndTime = endTime;
        segment.StartTCP = tcpPose;
        segment.EndTCP = tcpPose;
        segment.QStart = qPose;
        segment.QEnd = qPose;
        int n = qPose.Length;
        segment.Interpolations = new IInterpolation[n];
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
        TrajectorySegment segment = new();
        segment.Type = type;
        segment.StartTime = startTime;
        segment.EndTime = endTime;
        segment.StartTCP = startTCP;
        segment.EndTCP = endTCP;
        segment.QStart = qStart;
        segment.QEnd = qEnd;
        int n = Mathf.Max(qStart.Length, qEnd.Length);
        segment.Interpolations = new IInterpolation[n];
        for (int i = 0; i < n; i++)
        {
            LinearInterpolation interpolation = new();
            interpolation.Build(qStart[i], qEnd[i], endTime - startTime);
            segment.Interpolations[i] = interpolation;
        }
        return segment;
    }

    public static TrajectorySegment BuildQuinticSegment(
        TrajectorySegmentType type,
        float startTime,
        float endTime,
        Pose startTCP,
        Pose endTCP,
        float[] qStart,
        float[] qEnd,
        float[] vStart = null,
        float[] vEnd = null,
        float[] aStart = null,
        float[] aEnd = null)
    {
        TrajectorySegment segment = new();
        segment.Type = type;
        segment.StartTime = startTime;
        segment.EndTime = endTime;
        segment.StartTCP = startTCP;
        segment.EndTCP = endTCP;
        segment.QStart = qStart;
        segment.QEnd = qEnd;

        int n = Mathf.Max(qStart.Length, qEnd.Length);
        float duration = Mathf.Max(1e-5f, endTime - startTime);
        segment.Interpolations = new IInterpolation[n];

        for (int i = 0; i < n; i++)
        {
            QuinticPolynomial interpolation = new();
            float jointVStart = vStart != null && i < vStart.Length ? vStart[i] : 0f;
            float jointVEnd = vEnd != null && i < vEnd.Length ? vEnd[i] : 0f;
            float jointAStart = aStart != null && i < aStart.Length ? aStart[i] : 0f;
            float jointAEnd = aEnd != null && i < aEnd.Length ? aEnd[i] : 0f;

            interpolation.Build(
                qStart[i],
                qEnd[i],
                jointVStart,
                jointVEnd,
                jointAStart,
                jointAEnd,
                duration);
            segment.Interpolations[i] = interpolation;
        }

        return segment;
    }
}
