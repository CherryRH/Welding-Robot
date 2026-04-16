using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// TCP 路径可视化器
/// </summary>
public class TcpPathVisualizer : MonoBehaviour
{
    public GameObject SamplePoint;

    private Dictionary<TcpPathPoint, GameObject> tcpPathPoints = new();

    private Dictionary<WeldPointData, GameObject> weldPoints = new();

    void Start()
    {

    }

    void Update()
    {

    }

    public void ShowTcpPathPoints(TcpPathPlanner tcpPathPlanner)
    {
        // 清除旧数据
        Clear();
        int i = 0;
        foreach (TcpPathPoint item in tcpPathPlanner.Points)
        {
            Pose pose = item.Pose;
            // 实例化采样点
            GameObject point = Instantiate(
                SamplePoint,
                MathUtil.D2UPosition(pose.position),
                MathUtil.D2URotation(pose.rotation),
                transform
            );
            point.name = $"TcpPathPoint_{item.Type}_{++i}";
            tcpPathPoints.Add(item, point);
        }
    }

    /// <summary>
    /// 显示焊接点数据列表（用于 Replay 状态）
    /// </summary>
    public void ShowWeldPointData(List<WeldPointData> points)
    {
        Clear();
        int i = 0;
        foreach (var item in points)
        {
            Pose pose = item.TcpPose;
            GameObject point = Instantiate(
                SamplePoint,
                MathUtil.D2UPosition(pose.position),
                MathUtil.D2URotation(pose.rotation),
                transform
            );
            point.name = $"WeldPoint_{item.Type}_{++i}";
            weldPoints.Add(item, point);
        }
    }

    public void Clear()
    {
        // 清除所有点
        foreach (var item in tcpPathPoints)
        {
            Destroy(item.Value);
        }
        tcpPathPoints.Clear();
        foreach (var item in weldPoints)
        {
            Destroy(item.Value);
        }
        weldPoints.Clear();
    }
}
