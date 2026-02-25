using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// TCP路径规划器
/// </summary>
public class TcpPathPlanner
{
    /// <summary>
    /// TCP路径点列表
    /// </summary>
    public LinkedList<TcpPathPoint> Points = new();

    /// <summary>
    /// 当前规划到的TCP路径节点
    /// </summary>
    public LinkedListNode<TcpPathPoint> CurrentNode { get; private set; } = null;

    /// <summary>
    /// 路径规划器
    /// </summary>
    private ApproachPathPlanner approachPlanner = new();
    private WeldPathPlanner weldPlanner = new();
    private AdjustPathPlanner adjustPlanner = new();

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

    private RobotModel robot;

    public void Init(RobotModel robot)
    {
        this.robot = robot;
        approachPlanner.Init(robot);
        weldPlanner.Init(robot);
        adjustPlanner.Init(robot);
    }

    public void Plan(WeldTask task)
    {
        // 从机械臂位置开始，按照焊缝顺序规划TCP路径点
        Clear();
        Pose currentPose = robot.TCPPose;
        foreach (var seam in task.WeldSeams)
        {
            // 规划焊缝路径点
            List<TcpPathPoint> weldSeamPoints = weldPlanner.Plan(seam);
            // 路径点过少，跳过焊缝
            if (weldSeamPoints == null || weldSeamPoints.Count < 2)
                continue;
            // 规划接近路径点，取焊缝的起点姿态
            Pose targetPose = weldSeamPoints[0].Pose;
            List<TcpPathPoint> approachPoints = approachPlanner.Plan(currentPose, targetPose, seam);
            // 插入路径
            foreach (var item in approachPoints)
            {
                Points.AddLast(item);
            }
            foreach (var item in weldSeamPoints)
            {
                Points.AddLast(item);
            }
            // 更新当前姿态
            currentPose = weldSeamPoints[weldSeamPoints.Count-1].Pose;
        }
        // 设置当前节点
        CurrentNode = Points.First;
    }

    public List<TcpPathPoint> GetPathPart()
    {
        // 获取下一部分路径点
        List<TcpPathPoint> result = new();
        LinkedListNode<TcpPathPoint> node = CurrentNode;
        while (node != null)
        {
            var point = node.Value;
            result.Add(point);
            node = node.Next;
            if (point.Flag == TcpPathPoint.PointFlag.End)
            {
                break;
            }
        }
        return result;
    }

    public void HandleTrajectoryPlanResult(TrajectoryPlanResult result)
    {
        if (result == null)
            return;

        // 处理轨迹规划结果
        switch (result.PlanStatus)
        {
            case TrajectoryPlanResult.TrajectoryPlanStatus.Ok:
                // 推进当前规划节点到下一段路径
                while (CurrentNode != null)
                {
                    var point = CurrentNode.Value;
                    CurrentNode = CurrentNode.Next;
                    if (point.Flag == TcpPathPoint.PointFlag.End)
                    {
                        break;
                    }
                }
                // 全部规划完
                if (CurrentNode == null)
                {
                    Status = PlanStatus.Suceeded;
                }
                break;

            case TrajectoryPlanResult.TrajectoryPlanStatus.JointSpeedLimitViolated:
                // 推进当前规划节点到调姿点，截断当前路径，并插入调姿路径
                while (CurrentNode != null)
                {
                    TcpPathPoint point = CurrentNode.Value;
                    if (point == result.CurrentPoint)
                    {
                        // 截断当前路径
                        point.Flag = TcpPathPoint.PointFlag.End;
                        // 规划调姿路径
                        List<TcpPathPoint> adjustPoints = adjustPlanner.Plan(point.Pose, point.Seam);
                        LinkedListNode<TcpPathPoint> insertNode = CurrentNode;
                        foreach (var item in adjustPoints)
                        {
                            Points.AddAfter(insertNode, item);
                            insertNode = insertNode.Next;
                        }
                        // 插入一个新的起点
                        TcpPathPoint newStartPoint = new(
                            point.Pose,
                            point.Type,
                            TcpPathPoint.PointFlag.Start,
                            point.Seam,
                            point.Speed);
                        Points.AddAfter(insertNode, newStartPoint);
                        break;
                    }
                    else
                    {
                        CurrentNode = CurrentNode.Next;
                    }
                }
                break;

            default:
                Status = PlanStatus.Failed;
                Debug.LogWarning($"Trajectory planning failed: {result.PlanStatus}. ");
                break;
        }
    }

    public void Clear()
    {
        Points.Clear();
        CurrentNode = null;
        Status = PlanStatus.Unfinished;
    }
}
