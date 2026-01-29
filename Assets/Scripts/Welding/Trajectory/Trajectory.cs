using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 仿真路径队列（在线规划）
/// </summary>
public class Trajectory
{
    private Queue<TrajectorySegment> segmentQueue;

    public float[] Evaluate(float time)
    {
        return new float[0];
    }
}

/// <summary>
/// 路径片段
/// </summary>
public class TrajectorySegment
{
    public float StartTime;
    public float EndTime;

    public Pose StartTCP;
    public Pose EndTCP;

    public float[] QStart;
    public float[] QEnd;

    public PathInterpolation[] Interpolations;
}
