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
        int totalApproachPoints = 0;
        int totalWeldPoints = 0;

        for (int i = 0; i < task.WeldSeams.Count; i++)
        {
            var seam = task.WeldSeams[i];

            // 规划焊接路径点
            List<TcpPathPoint> weldSeamPoints = weldPlanner.Plan(seam);
            // 路径点数量不足，跳过
            if (weldSeamPoints == null || weldSeamPoints.Count < 2)
            {
                Debug.LogWarning($"[TcpPath] Seam {i}: weld path too short ({weldSeamPoints?.Count ?? 0} pts), skipped.");
                continue;
            }

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

            totalApproachPoints += approachPoints.Count;
            totalWeldPoints += weldSeamPoints.Count;

            // 更新当前姿态
            currentPose = weldSeamPoints[weldSeamPoints.Count - 1].Pose;
        }

        // 置位当前节点
        TaskState.CurrentNode = Points.First;

        Debug.Log($"[TcpPath] Plan complete: {task.WeldSeams.Count} seams, " +
            $"{totalApproachPoints} approach pts + {totalWeldPoints} weld pts = {Points.Count} total.");
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

        if (result.Count > 0)
            Debug.Log($"[TcpPath] GetPathPart: {result.Count} pts, " +
                $"{result[0].Type}/{result[0].Flag} → {result[^1].Type}/{result[^1].Flag}");

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
                Debug.LogWarning($"[TcpPath] Joint speed limit violated at " +
                    $"{result.CurrentPoint.Type}/{result.CurrentPoint.Flag}, inserting adjust path.");
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

            case TrajectoryPlanResult.TrajectoryPlanStatus.TcpPositionUnreachable:
                TaskState.Status = WeldTaskPlanState.PlanStatus.Failed;
                Debug.LogError($"[TcpPath] TCP unreachable at " +
                    $"{result.CurrentPoint?.Type}/{result.CurrentPoint?.Flag} " +
                    $"pos=({result.CurrentPoint?.Pose.position.x:F3}, " +
                    $"{result.CurrentPoint?.Pose.position.y:F3}, " +
                    $"{result.CurrentPoint?.Pose.position.z:F3}). Task FAILED.");
                break;

            default:
                Debug.LogWarning($"[TcpPath] Unknown plan status: {result.PlanStatus}");
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
        if (currentSegment == null)
        {
            Debug.LogWarning("[TcpPath] ReplanFromPosition: currentSegment is null, abort.");
            return;
        }

        TcpPathPoint startPoint = currentSegment.StartPoint;
        TcpPathPoint endPoint = currentSegment.EndPoint;

        // ===== Step 1：定位 EndPoint 节点 =====
        LinkedListNode<TcpPathPoint> endNode = FindNode(endPoint);
        if (endNode == null)
        {
            Debug.LogWarning("[TcpPath] ReplanFromPosition: endNode not found in Points, abort.");
            return;
        }

        // ===== Step 2：向前遍历，找到 Approach End（第一个 Flag=End 的节点）=====
        LinkedListNode<TcpPathPoint> approachEndNode = endNode;
        while (approachEndNode != null)
        {
            if (approachEndNode.Value.Flag == TcpPathPoint.PointFlag.End)
                break;
            approachEndNode = approachEndNode.Next;
        }
        if (approachEndNode == null)
        {
            Debug.LogWarning("[TcpPath] ReplanFromPosition: no End-flag node found after endNode, abort.");
            return;
        }

        Pose targetPose = approachEndNode.Value.Pose;
        WeldSeam targetSeam = approachEndNode.Value.Seam;

        Debug.Log($"[TcpPath] Replan: truncating from " +
            $"({endPoint.Type}/{endPoint.Flag}) to ({approachEndNode.Value.Type}/{approachEndNode.Value.Flag}), " +
            $"target=({targetPose.position.x:F3}, {targetPose.position.y:F3}, {targetPose.position.z:F3})");

        // ===== Step 3：在 StartPoint 之后插入截断 End 点 =====
        TcpPathPoint truncateEnd = new(
            robot.TCPPose,
            TcpPathPoint.PointType.Approach,
            TcpPathPoint.PointFlag.End,
            targetSeam,
            robot.Config.TCPMaxSpeed);

        LinkedListNode<TcpPathPoint> startNode = FindNode(startPoint);
        if (startNode == null)
        {
            Debug.LogWarning("[TcpPath] ReplanFromPosition: startNode not found in Points, abort.");
            return;
        }

        LinkedListNode<TcpPathPoint> insertAfter = Points.AddAfter(startNode, truncateEnd);

        // ===== Step 4：删除 EndPoint 到 ApproachEnd（含）的所有节点 =====
        int removedCount = 0;
        LinkedListNode<TcpPathPoint> removeNode = endNode;
        while (removeNode != null && removeNode != approachEndNode)
        {
            LinkedListNode<TcpPathPoint> next = removeNode.Next;
            Points.Remove(removeNode);
            removeNode = next;
            removedCount++;
        }
        // 删除 approachEndNode
        if (removeNode == approachEndNode)
        {
            Points.Remove(approachEndNode);
            removedCount++;
        }

        // ===== Step 5：重新规划接近路径并插入 =====
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

        Debug.Log($"[TcpPath] Replan done: removed {removedCount} nodes, " +
            $"inserted {newApproach.Count} new approach pts. " +
            $"List size now {Points.Count}.");
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
