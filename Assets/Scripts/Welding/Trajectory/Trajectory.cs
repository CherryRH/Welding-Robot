using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 仿真路径队列（在线规划）
/// </summary>
public class Trajectory
{
    private LinkedList<TrajectorySegment> segments = new();

    public TrajectorySegment CurrentSegment { get; private set; }

    public TrajectorySegment LastSegment => segments.Count > 0 ? segments.Last.Value : CurrentSegment;

    public bool HasSegment => CurrentSegment != null || segments.Count > 0;

    public bool UnderHighWaterMark => segments.Count <= 20;
    public bool OverLowWaterMark => segments.Count >= 10;

    public float[] Evaluate(float simTime)
    {
        // 如果当前没有执行段，尝试取一个
        if (CurrentSegment == null)
        {
            if (segments.Count == 0)
                return null;

            CurrentSegment = segments.First.Value;
            segments.RemoveFirst();
        }

        TrajectorySegment seg = CurrentSegment;

        // 如果时间还没到这个段开始（理论上不该发生，防御一下）
        if (simTime <= seg.StartTime)
            return seg.QStart;

        // 如果已经超过当前段结束时间
        if (simTime >= seg.EndTime)
        {
            float[] qEnd = seg.QEnd;

            // 切换到下一段
            CurrentSegment = null;

            return qEnd;
        }

        return seg.Evaluate(simTime);
    }

    public void Add(TrajectorySegment segment)
    {
        segments.AddLast(segment);
    }

    public void Clear()
    {
        segments.Clear();
        CurrentSegment = null;
    }
}
