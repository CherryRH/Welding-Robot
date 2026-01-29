using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 工件绑定
/// </summary>
public class WorkpieceBinder : MonoBehaviour
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
        // 获取工件原点位置（数据坐标系）
        return Origin != null ? MathUtil.UnityToDataPosition(Origin.position) : Vector3.zero;
    }
}
