using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接任务规划的状态
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
    /// 当前规划到的焊缝
    /// </summary>
    public WeldSeam CurrentWeldSeam => CurrentNode?.Value.Seam;

    /// <summary>
    /// 当前规划到的TCP路径节点
    /// </summary>
    public LinkedListNode<TcpPathPoint> CurrentNode = null;

    /// <summary>
    /// 一次任务中同一位置重复调姿的上限
    /// </summary>
    private const int MaxAdjustCount = 3;
    private int currentAdjustCount = 0;
    private WeldSeam lastAdjustWeldSeam = null;
    private Pose lastAdjustPose = new();

    public void ToNextPath()
    {
        // 前进到下一组路径
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
        // 前进到指定路径点
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

    public void CheckAdjustCount(TrajectoryPlanResult result)
    {
        // 检查重规划次数
        if (CurrentNode == null || result == null) return;

        TcpPathPoint point = result.CurrentPoint;
        
        if (result.PlanStatus != TrajectoryPlanResult.TrajectoryPlanStatus.Ok)
        {
            if (point.Seam == lastAdjustWeldSeam && MathUtil.IsPoseEqual(point.Pose, lastAdjustPose))
            {
                currentAdjustCount++;
            }
            else
            {
                currentAdjustCount = 1;
                lastAdjustWeldSeam = point.Seam;
                lastAdjustPose = point.Pose;
            }
        }
        if (currentAdjustCount > MaxAdjustCount)
        {
            Status = PlanStatus.Failed;
        }
    }

    public void Reset()
    {
        Status = PlanStatus.Unfinished;
        CurrentNode = null;
        currentAdjustCount = 0;
        lastAdjustWeldSeam = null;
        lastAdjustPose = new();
    }
}
