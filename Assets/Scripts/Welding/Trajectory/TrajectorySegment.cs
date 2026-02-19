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
        Adjust
    }
    public TrajectorySegmentType Type;

    public float StartTime;
    public float EndTime;

    public float[] QStart;
    public float[] QEnd;

    public IJointInterpolator Interpolation;

    public TrajectorySegment(
        TrajectorySegmentType type,
        float startTime, float endTime,
        float[] qStart, float[] qEnd,
        IJointInterpolator interpolation)
    {
        Type = type;
        StartTime = startTime;
        EndTime = endTime;
        QStart = qStart;
        QEnd = qEnd;
        Interpolation = interpolation;
    }

    public float[] Evaluate(float simTime)
    {
        if (Interpolation == null) return new float[QStart.Length];
        // 计算 simTime 时刻的关节位置
        float t = Mathf.Clamp(simTime - StartTime, 0f, EndTime - StartTime);
        return Interpolation.Evaluate(t);
    }
}
