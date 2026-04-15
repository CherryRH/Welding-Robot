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
    /// 一次任务中同一位置重复规划的上限
    /// </summary>
    private const int MaxReplanCount = 3;
    private int currentReplanCount = 0;
    private WeldSeam lastReplanWeldSeam = null;
    private Pose lastReplanPose = new();

    // ===== 运行时重规划状态 =====

    /// <summary>是否正在重规划过程中</summary>
    public bool IsReplanning = false;

    /// <summary>重规划冷却计时器（秒）</summary>
    public float ReplanCooldown = 0f;

    /// <summary>两次重规划的最小间隔（秒）</summary>
    public const float ReplanCooldownDuration = 1.0f;

    /// <summary>连续处于 Warning 级别的帧数</summary>
    public int ConsecutiveWarnings = 0;

    /// <summary>连续 Warning 帧数上限，超过后判定为狭窄空间</summary>
    public const int MaxConsecutiveWarnings = 100;

    /// <summary>狭窄空间模式：允许在 Warning 下继续运行</summary>
    public bool NarrowSpaceMode = false;

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

        // 重规划状态重置
        IsReplanning = false;
        ReplanCooldown = 0f;
        ConsecutiveWarnings = 0;
        NarrowSpaceMode = false;
    }
}
