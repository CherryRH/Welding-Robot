using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 碰撞箱可视化器（Play 模式下持续渲染线框）
/// 挂载到场景中任意 GameObject 上，通过 Add/Remove 方法管理需要绘制的碰撞箱
/// </summary>
public class ColliderVisualizer : MonoBehaviour
{
    // -------------------------------------------------------
    // 内部数据结构
    // -------------------------------------------------------

    private struct BoxEntry
    {
        public Vector3 Center;      // 世界坐标中心
        public Vector3 Size;        // 世界坐标尺寸（已含 lossyScale）
        public Quaternion Rotation; // 世界坐标旋转
        public Color Color;
    }

    // -------------------------------------------------------
    // 公开配置
    // -------------------------------------------------------

    [Tooltip("默认线框颜色")]
    public Color DefaultColor = new Color(0f, 1f, 0f, 0.9f);

    // -------------------------------------------------------
    // 私有状态
    // -------------------------------------------------------

    private readonly Dictionary<int, BoxEntry> entries = new();
    private int nextId = 0;

    private Material lineMaterial;

    // -------------------------------------------------------
    // Unity 生命周期
    // -------------------------------------------------------

    void Awake()
    {
        // 创建 GL 所需的无光照材质
        Shader shader = Shader.Find("Hidden/Internal-Colored");
        lineMaterial = new Material(shader)
        {
            hideFlags = HideFlags.HideAndDontSave
        };
        lineMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        lineMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
        lineMaterial.SetInt("_ZWrite", 0);
    }

    void OnDestroy()
    {
        if (lineMaterial != null)
            Destroy(lineMaterial);
    }

    void OnRenderObject()
    {
        if (entries.Count == 0 || lineMaterial == null) return;

        lineMaterial.SetPass(0);
        GL.PushMatrix();
        GL.MultMatrix(Matrix4x4.identity); // 世界坐标

        foreach (var kv in entries)
        {
            DrawBox(kv.Value);
        }

        GL.PopMatrix();
    }

    // -------------------------------------------------------
    // 公开 API
    // -------------------------------------------------------

    /// <summary>
    /// 从 BoxCollider 添加可视化条目，自动读取世界变换
    /// </summary>
    /// <param name="collider">目标 BoxCollider</param>
    /// <param name="color">线框颜色，null 则使用 DefaultColor</param>
    /// <returns>条目 ID，用于后续 Remove</returns>
    public int Add(BoxCollider collider, Color? color = null)
    {
        if (collider == null) return -1;

        Transform t = collider.transform;
        // BoxCollider.center 是局部空间偏移，需转换到世界坐标
        Vector3 worldCenter = t.TransformPoint(collider.center);
        // lossyScale 已包含父级缩放链
        Vector3 worldSize = Vector3.Scale(collider.size, t.lossyScale);

        return AddEntry(worldCenter, worldSize, t.rotation, color ?? DefaultColor);
    }

    /// <summary>
    /// 从 BoxCollider 添加可视化条目，并绑定到 Transform 使其每帧自动跟随
    /// （适用于机械臂等运动物体）
    /// </summary>
    /// <param name="collider">目标 BoxCollider</param>
    /// <param name="color">线框颜色，null 则使用 DefaultColor</param>
    /// <returns>条目 ID</returns>
    public int AddTracked(BoxCollider collider, Color? color = null)
    {
        if (collider == null) return -1;
        int id = Add(collider, color);
        if (id >= 0)
            StartCoroutine(TrackCollider(id, collider));
        return id;
    }

    /// <summary>
    /// 直接指定世界坐标中心、尺寸、旋转添加条目
    /// （适用于不依赖 BoxCollider 的纯数据碰撞箱）
    /// </summary>
    public int Add(Vector3 worldCenter, Vector3 worldSize, Quaternion worldRotation, Color? color = null)
    {
        return AddEntry(worldCenter, worldSize, worldRotation, color ?? DefaultColor);
    }

    /// <summary>
    /// 移除指定 ID 的可视化条目
    /// </summary>
    public void Remove(int id)
    {
        entries.Remove(id);
    }

    /// <summary>
    /// 清除所有可视化条目
    /// </summary>
    public void Clear()
    {
        entries.Clear();
    }

    /// <summary>
    /// 更新指定 ID 条目的颜色
    /// </summary>
    public void SetColor(int id, Color color)
    {
        if (entries.TryGetValue(id, out var entry))
        {
            entry.Color = color;
            entries[id] = entry;
        }
    }

    // -------------------------------------------------------
    // 内部实现
    // -------------------------------------------------------

    private int AddEntry(Vector3 center, Vector3 size, Quaternion rotation, Color color)
    {
        int id = nextId++;
        entries[id] = new BoxEntry
        {
            Center = center,
            Size = size,
            Rotation = rotation,
            Color = color
        };
        return id;
    }

    private System.Collections.IEnumerator TrackCollider(int id, BoxCollider collider)
    {
        // 每帧同步 BoxCollider 的世界变换到对应条目
        while (collider != null && entries.ContainsKey(id))
        {
            Transform t = collider.transform;
            if (entries.TryGetValue(id, out var entry))
            {
                entry.Center = t.TransformPoint(collider.center);
                entry.Size = Vector3.Scale(collider.size, t.lossyScale);
                entry.Rotation = t.rotation;
                entries[id] = entry;
            }
            yield return null;
        }
    }

    private void DrawBox(BoxEntry box)
    {
        // 计算 8 个顶点（局部坐标，再旋转到世界坐标）
        Vector3 h = box.Size * 0.5f;

        // 局部空间 8 顶点
        Vector3[] local = new Vector3[8]
        {
            new(-h.x, -h.y, -h.z),
            new( h.x, -h.y, -h.z),
            new( h.x,  h.y, -h.z),
            new(-h.x,  h.y, -h.z),
            new(-h.x, -h.y,  h.z),
            new( h.x, -h.y,  h.z),
            new( h.x,  h.y,  h.z),
            new(-h.x,  h.y,  h.z),
        };

        // 转换到世界坐标
        Vector3[] w = new Vector3[8];
        for (int i = 0; i < 8; i++)
            w[i] = box.Center + box.Rotation * local[i];

        GL.Begin(GL.LINES);
        GL.Color(box.Color);

        // 底面
        DrawLine(w[0], w[1]);
        DrawLine(w[1], w[2]);
        DrawLine(w[2], w[3]);
        DrawLine(w[3], w[0]);
        // 顶面
        DrawLine(w[4], w[5]);
        DrawLine(w[5], w[6]);
        DrawLine(w[6], w[7]);
        DrawLine(w[7], w[4]);
        // 四条竖边
        DrawLine(w[0], w[4]);
        DrawLine(w[1], w[5]);
        DrawLine(w[2], w[6]);
        DrawLine(w[3], w[7]);

        GL.End();
    }

    private static void DrawLine(Vector3 a, Vector3 b)
    {
        GL.Vertex(a);
        GL.Vertex(b);
    }
}