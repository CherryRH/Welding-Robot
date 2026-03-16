using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 工作台绑定
/// </summary>
public class WorkbenchBinder : MonoBehaviour
{
    public Transform Origin;

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public Vector3 GetOriginPoint()
    {
        // 获取工作台原点位置（数据坐标系）
        return Origin != null ? MathUtil.UnityToDataPosition(Origin.position) : Vector3.zero;
    }
}
