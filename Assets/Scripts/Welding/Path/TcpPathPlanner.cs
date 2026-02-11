using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Plans full TCP path (approach + weld seams) and advances by seam blocks.
/// </summary>
public class TcpPathPlanner
{
    /// <summary>
    /// Full path points to be planned/executed.
    /// </summary>
    public LinkedList<TcpPathPoint> Points = new();

    /// <summary>
    /// Current planning cursor.
    /// </summary>
    private LinkedListNode<TcpPathPoint> currentNode = null;

    private ApproachPathPlanner approachPlanner = new();
    private WeldPathPlanner weldPlanner = new();

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
        Clear();
        Pose currentPose = robot.TCPPose;
        foreach (var seam in task.WeldSeams)
        {
            List<TcpPathPoint> weldSeamPoints = weldPlanner.Plan(seam);
            if (weldSeamPoints == null || weldSeamPoints.Count < 2)
                continue;

            Pose targetPose = weldSeamPoints[0].Pose;
            List<TcpPathPoint> approachPoints = approachPlanner.Plan(currentPose, targetPose, seam);

            foreach (var item in approachPoints)
                Points.AddLast(item);
            foreach (var item in weldSeamPoints)
                Points.AddLast(item);

            currentPose = weldSeamPoints[weldSeamPoints.Count - 1].Pose;
        }

        currentNode = Points.First;
    }

    public List<TcpPathPoint> GetPathPart()
    {
        List<TcpPathPoint> result = new();
        LinkedListNode<TcpPathPoint> node = currentNode;
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
        if (result == null)
            return;

        switch (result.PlanStatus)
        {
            case TrajectoryPlanResult.TrajectoryPlanStatus.Ok:
                while (currentNode != null)
                {
                    var point = currentNode.Value;
                    currentNode = currentNode.Next;
                    if (point.Flag == TcpPathPoint.PointFlag.End)
                        break;
                }

                if (currentNode == null)
                    Status = PlanStatus.Suceeded;
                break;

            default:
                Status = PlanStatus.Failed;
                Debug.LogError($"Trajectory planning failed: {result.PlanStatus}. " +
                    $"From seam {result.StartPoint?.SeamId} to seam {result.EndPoint?.SeamId}.");
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
