using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 维护焊接任务的规划状态
/// </summary>
public class WeldTaskPlanState
{
    /// <summary>
    /// 规划状态
    /// </summary>
    public enum PlanStatus
    {
        Unfinished,
        Suceeded,
        Failed
    }
    public PlanStatus Status = PlanStatus.Unfinished;

    /// <summary>
    /// 当前规划的焊缝
    /// </summary>
    public WeldSeam CurrentWeldSeam => CurrentNode?.Value.Seam;

    /// <summary>
    /// 当前规划到的TCP路径节点
    /// </summary>
    public LinkedListNode<TcpPathPoint> CurrentNode = null;

    /// <summary>
    /// 一个焊缝上同一个点的最大重规划次数
    /// </summary>
    private const int MaxReplanCount = 3;
    private int currentReplanCount = 0;
    private WeldSeam lastReplanWeldSeam = null;
    private Pose lastReplanPose = new();

    public void ToNextPath()
    {
        // 推进到下一段路径
        LinkedListNode<TcpPathPoint> node = CurrentNode;
        while (node != null)
        {
            var point = node.Value;
            node = node.Next;
            if (point.Flag == TcpPathPoint.PointFlag.End)
            {
                break;
            }
        }
        CurrentNode = node;
        // 全部规划完
        if (CurrentNode == null)
        {
            Status = PlanStatus.Suceeded;
        }
    }

    public void ToPoint(TcpPathPoint point)
    {
        // 推进到指定的路径点
        LinkedListNode<TcpPathPoint> node = CurrentNode;
        while (node != null)
        {
            if (node.Value == point)
            {
                break;
            }
            else
            {
                node = node.Next;
            }
        }
        if (node.Value == point)
        {
            CurrentNode = node;
        }
    }

    public void CheckReplanCount(TrajectoryPlanResult result)
    {
        // 检查重规划次数
        if (CurrentNode == null || result == null) return;

        TcpPathPoint point = result.CurrentPoint;
        
        if (result.PlanStatus != TrajectoryPlanResult.TrajectoryPlanStatus.Ok)
        {
            if (point.Seam == lastReplanWeldSeam && MathUtil.IsPoseEqual(point.Pose, lastReplanPose))
            {
                currentReplanCount++;
            }
            else
            {
                currentReplanCount = 1;
                lastReplanWeldSeam = point.Seam;
                lastReplanPose = point.Pose;
            }
        }
        if (currentReplanCount > MaxReplanCount)
        {
            Status = PlanStatus.Failed;
        }
    }

    public void Reset()
    {
        Status = PlanStatus.Unfinished;
        CurrentNode = null;
        currentReplanCount = 0;
        lastReplanWeldSeam = null;
        lastReplanPose = new();
    }
}
