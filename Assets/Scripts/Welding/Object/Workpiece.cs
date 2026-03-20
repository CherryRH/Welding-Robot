using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 工件
/// </summary>
public class Workpiece
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

    /// <summary>
    /// 碰撞箱列表
    /// </summary>
    public List<Collider> Colliders = new();

    public Workpiece(WorkpieceData data)
    {
        FileName = data.FileName;
        Position = data.Position;
        Rotation = data.Rotation;
        Scale = data.Scale;
        data.Colliders?.ForEach(c => Colliders.Add(new Collider(c)));
    }

    /// <summary>
    /// AABB碰撞箱
    /// </summary>
    public class Collider
    {
        public Vector3 Min;
        public Vector3 Max;
        public Collider(Vector3 min, Vector3 max)
        {
            Min = min;
            Max = max;
        }

        public Collider(WorkpieceData.ColliderData data)
        {
            Min = data.Min;
            Max = data.Max;
        }
    }
}
