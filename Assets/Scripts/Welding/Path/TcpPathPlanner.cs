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
    private LinkedListNode<TcpPathPoint> currentNode = null;

    /// <summary>
    /// 路径规划器
    /// </summary>
    private ApproachPathPlanner approachPlanner = new();
    private WeldPathPlanner weldPlanner = new();

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
        currentNode = Points.First;
    }

    public List<TcpPathPoint> GetPathPart()
    {
        // 获取下一部分路径点
        List<TcpPathPoint> result = new();
        LinkedListNode<TcpPathPoint> node = currentNode;
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
        // 处理轨迹规划结果
        switch (result.PlanStatus)
        {
            case TrajectoryPlanResult.TrajectoryPlanStatus.Ok:
                // 推进当前规划节点到下一段路径
                while (currentNode != null)
                {
                    var point = currentNode.Value;
                    currentNode = currentNode.Next;
                    if (point.Flag == TcpPathPoint.PointFlag.End)
                    {
                        break;
                    }
                }
                // 全部规划完
                if (currentNode == null)
                {
                    Status = PlanStatus.Suceeded;
                }
                break;
        }
    }

    public void Clear()
    {
        Points.Clear();
        currentNode = null;
        Status = PlanStatus.Unfinished;
    }
}
