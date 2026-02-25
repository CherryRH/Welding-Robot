using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// TCP路径可视化器
/// </summary>
public class TcpPathVisualizer : MonoBehaviour
{
    public GameObject SamplePoint;

    private Dictionary<TcpPathPoint, GameObject> samplePoints = new();

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void ShowTcpPathPoints(TcpPathPlanner tcpPathPlanner)
    {
        // 显示采样点
        Clear();
        int i = 0;
        foreach (TcpPathPoint item in tcpPathPlanner.Points)
        {
            Pose pose = item.Pose;
            // 实例化采样点
            GameObject point = Instantiate(
                SamplePoint,
                MathUtil.DataToUnityPosition(pose.position),
                MathUtil.DataToUnityRotation(pose.rotation),
                transform
            );
            point.name = $"TcpPathPoint_{item.Type}_{++i}";
            samplePoints.Add(item, point);
        }
    }

    public void Clear()
    {
        // 清除采样点
        foreach (var item in samplePoints)
        {
            Destroy(item.Value);
        }
        samplePoints.Clear();
    }
}
