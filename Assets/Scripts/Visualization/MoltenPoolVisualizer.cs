using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 熔池可视化器 — 基于射线检测的焊道条带渲染
/// 
/// 原理：
///   每帧从 TCP 两侧发射射线，检测工件表面交点，
///   用交点对构造三角条带 Mesh，条带宽度与线速度成反比
///   （速度越快焊道越窄，速度越慢焊道越宽）。
/// 
/// 生命周期：
///   每段焊接（Weld 类型 TrajectorySegment）对应一个条带，
///   焊接开始时 StartStrip()，每帧 AddSample()，结束时 FinalizeStrip()。
/// </summary>
public class MoltenPoolVisualizer : MonoBehaviour
{
    // ============================================================
    // 参数（Inspector 可调）
    // ============================================================

    [Header("射线检测")]
    [SerializeField] private float rayLength = 0.1f;
    [SerializeField] private LayerMask workpieceLayer;

    [Header("焊道尺寸")]
    [SerializeField] private float baseRadius = 0.005f;
    [SerializeField] private float speedFactor = 10f;

    [Header("外观")]
    [SerializeField] private Color stripColor = new(0.9f, 0.9f, 0.9f, 0.8f);
    [SerializeField] private Material stripMaterial;

    // ============================================================
    // 内部状态
    // ============================================================

    /// <summary>
    /// TCP Transform 引用
    /// </summary>
    public Transform Tcp;

    /// <summary>
    /// 当前是否有活跃的焊接条带
    /// </summary>
    public bool IsStripActive => _currentStrip != null;

    private readonly List<MoltenPoolStrip> _strips = new();
    private MoltenPoolStrip _currentStrip;

    // ============================================================
    // 公共接口（供 SimulationContext 调用）
    // ============================================================

    /// <summary>
    /// 焊接段开始 — 创建新条带
    /// </summary>
    public void StartStrip()
    {
        _currentStrip = new MoltenPoolStrip(
            transform, stripMaterial, stripColor, baseRadius, speedFactor,
            rayLength, workpieceLayer);
        _strips.Add(_currentStrip);
    }

    /// <summary>
    /// 每帧采样 — 射线检测 + 追加顶点
    /// </summary>
    /// <param name="tcp">TCP Transform</param>
    /// <param name="tcpSpeed">当前 TCP 线速度标量（m/s）</param>
    public void AddSample(Transform tcp, float tcpSpeed)
    {
        if (_currentStrip == null) return;

        // 计算当前半径：速度越大，半径越小
        float r = baseRadius / (1f + tcpSpeed * speedFactor);

        // TCP 坐标系
        Vector3 tcpPos = tcp.position;
        Vector3 weldDir = tcp.forward;   // Z 轴 = 焊枪方向
        Vector3 sideDir = tcp.right;     // X 轴 = 侧面正上方

        // 两条射线：沿侧面偏移 ±r，方向为焊枪方向
        Vector3 origin1 = tcpPos + sideDir * r;
        Vector3 origin2 = tcpPos - sideDir * r;

        Vector3 hitPoint1 = CastRay(origin1, weldDir);
        Vector3 hitPoint2 = CastRay(origin2, weldDir);

        // 若两条射线都没命中，跳过本帧
        if (hitPoint1 == origin1 + weldDir * rayLength &&
            hitPoint2 == origin2 + weldDir * rayLength)
            return;

        // 命中点略微提升，避免 Z-fighting
        hitPoint1 += Vector3.up * 0.0001f;
        hitPoint2 += Vector3.up * 0.0001f;

        _currentStrip.AddSample(hitPoint1, hitPoint2);
    }

    /// <summary>
    /// 焊接段结束 — 冻结条带
    /// </summary>
    public void FinalizeStrip()
    {
        if (_currentStrip == null) return;
        _currentStrip = null;
    }

    /// <summary>
    /// 仿真重置 — 销毁所有条带
    /// </summary>
    public void Clear()
    {
        foreach (var strip in _strips)
            strip.Destroy();
        _strips.Clear();
        _currentStrip = null;
    }

    // ============================================================
    // 射线检测
    // ============================================================

    private Vector3 CastRay(Vector3 origin, Vector3 direction)
    {
        if (Physics.Raycast(origin, direction, out RaycastHit hit, rayLength, workpieceLayer))
            return hit.point;
        // 未命中时返回射线终点（用于诊断）
        return origin + direction * rayLength;
    }
}

/// <summary>
/// 单条焊道条带 — 管理一个动态 Mesh 的生命周期
/// </summary>
internal class MoltenPoolStrip
{
    private readonly GameObject _gameObject;
    private readonly MeshFilter _meshFilter;
    private readonly MeshRenderer _meshRenderer;
    private readonly Mesh _mesh;

    private readonly List<Vector3> _vertices = new();
    private readonly List<int> _triangles = new();
    private readonly List<Vector2> _uv = new();

    private readonly float _baseRadius;
    private readonly float _speedFactor;
    private readonly float _rayLength;
    private readonly LayerMask _workpieceLayer;

    private bool _hasPrev;
    private Vector3 _prevLeft;
    private Vector3 _prevRight;

    private static int _stripCounter;

    public MoltenPoolStrip(
        Transform parent, Material material, Color color,
        float baseRadius, float speedFactor,
        float rayLength, LayerMask workpieceLayer)
    {
        _baseRadius = baseRadius;
        _speedFactor = speedFactor;
        _rayLength = rayLength;
        _workpieceLayer = workpieceLayer;

        // 创建 GameObject
        _gameObject = new GameObject($"MoltenPoolStrip_{_stripCounter++}");
        _gameObject.transform.SetParent(parent, false);

        _meshFilter = _gameObject.AddComponent<MeshFilter>();
        _meshRenderer = _gameObject.AddComponent<MeshRenderer>();

        // 材质
        if (material != null)
        {
            _meshRenderer.material = new Material(material) { color = color };
        }
        else
        {
            // 默认简单材质
            _meshRenderer.material = new Material(Shader.Find("Unlit/Color")) { color = color };
        }

        _mesh = new Mesh { name = $"MoltenPool_{_stripCounter}" };
        _meshFilter.mesh = _mesh;

        _hasPrev = false;
    }

    /// <summary>
    /// 追加一对交点，构造三角片
    /// </summary>
    public void AddSample(Vector3 left, Vector3 right)
    {
        if (!_hasPrev)
        {
            // 第一对点，只记录，不构三角面
            _prevLeft = left;
            _prevRight = right;
            _hasPrev = true;
            return;
        }

        int vBase = _vertices.Count;

        // 添加 4 个顶点：prevLeft, prevRight, left, right
        _vertices.Add(_prevLeft);
        _vertices.Add(_prevRight);
        _vertices.Add(left);
        _vertices.Add(right);

        // UV：沿条带方向 V 从 0→1，宽度方向 U 从 0→1
        float v = (vBase / 2) * 0.05f;  // 每对顶点 V 递增 0.05
        _uv.Add(new Vector2(0f, v));
        _uv.Add(new Vector2(1f, v));
        _uv.Add(new Vector2(0f, v + 0.05f));
        _uv.Add(new Vector2(1f, v + 0.05f));

        // 两个三角形（顺时针）
        _triangles.Add(vBase);     // prevLeft
        _triangles.Add(vBase + 2); // left
        _triangles.Add(vBase + 1); // prevRight

        _triangles.Add(vBase + 1); // prevRight
        _triangles.Add(vBase + 2); // left
        _triangles.Add(vBase + 3); // right

        // 更新 Mesh
        _mesh.SetVertices(_vertices);
        _mesh.SetUVs(0, _uv);
        _mesh.SetTriangles(_triangles, 0);
        _mesh.RecalculateNormals();

        // 保存本帧端点
        _prevLeft = left;
        _prevRight = right;
    }

    /// <summary>
    /// 销毁条带 GameObject 和 Mesh
    /// </summary>
    public void Destroy()
    {
        if (_mesh != null) Object.Destroy(_mesh);
        if (_gameObject != null) Object.Destroy(_gameObject);
    }
}