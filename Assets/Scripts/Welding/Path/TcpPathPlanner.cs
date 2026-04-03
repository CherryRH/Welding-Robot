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

    public void Clear()
    {
        Points.Clear();
    }
}
