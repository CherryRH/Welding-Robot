using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// TCP路径可视化器
/// </summary>
public class TcpPathVisualizer : MonoBehaviour
{
    public GameObject SamplePoint;

    private List<GameObject> samplePoints = new();

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void ShowTcpPathPoints(TcpPathPlanner tcpPathPlanner, float duration)
    {
        // 显示采样点
        Clear();
        int i = 0;
        foreach (TcpPathPoint item in tcpPathPlanner.Points)
        {
            Pose pose = item.Pose;
            Vector3 unityPos = MathUtil.DataToUnityPosition(pose.position);
            Quaternion unityRot = MathUtil.DataToUnityRotation(pose.rotation);

            Debug.DrawRay(
                unityPos,
                unityRot * Vector3.forward * 0.01f,
                Color.red,
                duration
            );

            // 实例化采样点
            GameObject point = Instantiate(
                SamplePoint,
                unityPos,
                unityRot,
                transform
            );
            point.name = $"TcpPathPoint_{item.Type}_{++i}";
            samplePoints.Add(point);
        }
    }

    public void Clear()
    {
        // 清除采样点
        foreach (var point in samplePoints)
        {
            Destroy(point);
        }
        samplePoints.Clear();
    }
}
