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
