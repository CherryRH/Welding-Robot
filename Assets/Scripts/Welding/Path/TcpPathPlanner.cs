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
    /// 子规划器
    /// </summary>
    private ApproachPathPlanner approachPlanner = new();
    private WeldPathPlanner weldPlanner = new();
    private AdjustPathPlanner adjustPlanner = new();

    private RobotModel robot;
    private WeldTaskPlanState TaskState;

    public void Init(RobotModel robot, WeldTaskPlanState taskState)
    {
        this.robot = robot;
        this.TaskState = taskState;
        approachPlanner.Init(robot);
        weldPlanner.Init(robot);
        adjustPlanner.Init(robot);
    }

    public void Plan(WeldTask task, CollisionMonitor shadowCollisionMonitor, RobotBinder shadowRobotBinder)
    {
        // 从机械臂当前位置开始，按焊缝顺序规划TCP路径
        Clear();
        Pose currentPose = robot.TCPPose;
        foreach (var seam in task.WeldSeams)
        {
            // 规划焊接路径点
            List<TcpPathPoint> weldSeamPoints = weldPlanner.Plan(seam);
            // 路径点数量不足，跳过
            if (weldSeamPoints == null || weldSeamPoints.Count < 2)
                continue;
            // 规划接近路径，取首个焊接点姿态
            Pose targetPose = weldSeamPoints[0].Pose;
            List<TcpPathPoint> approachPoints = approachPlanner.Plan(
                currentPose, targetPose,
                shadowCollisionMonitor, shadowRobotBinder, 
                seam);

            // 追加路径
            foreach (var item in approachPoints)
                Points.AddLast(item);
            foreach (var item in weldSeamPoints)
                Points.AddLast(item);

            // 更新当前姿态
            currentPose = weldSeamPoints[weldSeamPoints.Count - 1].Pose;
        }
        // 置位当前节点
        TaskState.CurrentNode = Points.First;
    }

    public List<TcpPathPoint> GetPathPart()
    {
        // 获取下一段路径点
        List<TcpPathPoint> result = new();
        LinkedListNode<TcpPathPoint> node = TaskState.CurrentNode;
        while (node != null)
        {
            var point = node.Value;
            result.Add(point);
            node = node.Next;
            if (point.Flag == TcpPathPoint.PointFlag.End)
                break;
        }
        return result;
    }

    public void HandleTrajectoryPlanResult(TrajectoryPlanResult result)
    {
        if (result == null) return;

        switch (result.PlanStatus)
        {
            case TrajectoryPlanResult.TrajectoryPlanStatus.Ok:
                TaskState.ToNextPath();
                break;

            case TrajectoryPlanResult.TrajectoryPlanStatus.JointSpeedLimitViolated:
                TaskState.ToPoint(result.CurrentPoint);
                if (TaskState.CurrentNode != null && TaskState.CurrentNode.Value == result.CurrentPoint)
                {
                    TcpPathPoint point = TaskState.CurrentNode.Value;
                    // 截断当前路径
                    point.Flag = TcpPathPoint.PointFlag.End;
                    // 规划调整路径
                    List<TcpPathPoint> adjustPoints = adjustPlanner.Plan(point.Pose, point.Seam);
                    LinkedListNode<TcpPathPoint> insertNode = TaskState.CurrentNode;
                    foreach (var item in adjustPoints)
                    {
                        Points.AddAfter(insertNode, item);
                        insertNode = insertNode.Next;
                    }
                    // 插入新的起点
                    TcpPathPoint newStartPoint = new(
                        point.Pose, point.Type,
                        TcpPathPoint.PointFlag.Start,
                        point.Seam, point.Speed);
                    Points.AddAfter(insertNode, newStartPoint);
                }
                break;

            default:
                break;
        }

        TaskState.CheckReplanCount(result);
    }

    /// <summary>
    /// 从当前位置重规划接近路径（运行时重规划调用）
    /// 
    /// 优化流程：
    /// 1. 找到当前接近段的 End 节点
    /// 2. 将 CurrentNode 指向截断 End 点（当前实际 Pose）
    /// 3. 删除 EndPoint 到 ApproachEnd（含）之间的所有节点
    /// 4. 重新规划接近路径并插入
    /// 5. 链表自动连接剩余部分，无需显式保存和重新插入
    /// </summary>
    /// <param name="currentSegment">当前执行的轨迹段</param>
    /// <param name="shadowCollisionMonitor">影子机械臂碰撞检测器</param>
    /// <param name="shadowRobotBinder">影子机械臂绑定器</param>
    public void ReplanFromPosition(
        TrajectorySegment currentSegment,
        CollisionMonitor shadowCollisionMonitor,
        RobotBinder shadowRobotBinder)
    {
        if (currentSegment == null) return;

        TcpPathPoint startPoint = currentSegment.StartPoint;
        TcpPathPoint endPoint = currentSegment.EndPoint;

        // ===== Step 1：定位 EndPoint 节点 =====
        LinkedListNode<TcpPathPoint> endNode = FindNode(endPoint);
        if (endNode == null) return;

        // ===== Step 2：向前遍历，找到 Approach End（第一个 Flag=End 的节点）=====
        LinkedListNode<TcpPathPoint> approachEndNode = endNode;
        while (approachEndNode != null)
        {
            if (approachEndNode.Value.Flag == TcpPathPoint.PointFlag.End)
                break;
            approachEndNode = approachEndNode.Next;
        }
        if (approachEndNode == null) return;

        Pose targetPose = approachEndNode.Value.Pose;
        WeldSeam targetSeam = approachEndNode.Value.Seam;

        // ===== Step 3：在 StartPoint 之后插入截断 End 点 =====
        TcpPathPoint truncateEnd = new(
            robot.TCPPose,
            TcpPathPoint.PointType.Approach,
            TcpPathPoint.PointFlag.End,
            targetSeam,
            robot.Config.TCPMaxSpeed);

        LinkedListNode<TcpPathPoint> startNode = FindNode(startPoint);
        if (startNode == null) return;

        LinkedListNode<TcpPathPoint> insertAfter = Points.AddAfter(startNode, truncateEnd);

        // ===== Step 4：删除 EndPoint 到 ApproachEnd（含）的所有节点 =====
        LinkedListNode<TcpPathPoint> removeNode = endNode;
        while (removeNode != null && removeNode != approachEndNode)
        {
            LinkedListNode<TcpPathPoint> next = removeNode.Next;
            Points.Remove(removeNode);
            removeNode = next;
        }
        // 删除 approachEndNode
        if (removeNode == approachEndNode)
            Points.Remove(approachEndNode);

        // ===== Step 5：重新规划接近路径并插入 =====
        // 注意：链表现在是：startNode → truncateEnd → 原remaining部分
        // 在truncateEnd之后插入新接近路径即可
        List<TcpPathPoint> newApproach = approachPlanner.Plan(
            robot.TCPPose,
            targetPose,
            shadowCollisionMonitor,
            shadowRobotBinder,
            targetSeam,
            ApproachPathPlanner.ApproachStrategy.RRT);

        foreach (var point in newApproach)
            insertAfter = Points.AddAfter(insertAfter, point);

        // ===== Step 6：CurrentNode 指向新路径 Start 点 =====
        TaskState.CurrentNode = Points.Find(newApproach[0]);
    }

    /// <summary>
    /// 在 Points 链表中查找指定 TcpPathPoint 对应的节点
    /// </summary>
    private LinkedListNode<TcpPathPoint> FindNode(TcpPathPoint point)
    {
        for (LinkedListNode<TcpPathPoint> node = Points.First; node != null; node = node.Next)
        {
            if (node.Value == point)
                return node;
        }
        return null;
    }

    public void Clear()
    {
        Points.Clear();
    }
}
