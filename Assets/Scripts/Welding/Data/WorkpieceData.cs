using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 工件数据
/// </summary>
[System.Serializable]
public class WorkpieceData
{
    /// <summary>
    /// 模型文件名称
    /// </summary>
    public string FileName;

    /// <summary>
    /// 原点的坐标、旋转和缩放
    /// </summary>
    public Vector3 Position;
    public Vector3 Rotation;
    public Vector3 Scale;

    [System.Serializable]
    public class ColliderData
    {
        // AABB碰撞箱
        public Vector3 Min;
        public Vector3 Max;
    }
    /// <summary>
    /// 碰撞箱列表
    /// </summary>
    public List<ColliderData> Colliders;
}
