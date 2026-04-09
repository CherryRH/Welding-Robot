using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// 工作台绑定
/// </summary>
public class WorkbenchBinder : MonoBehaviour
{
    public Transform Origin;

    public GameObject Workpiece;

    public Material WorkpieceMaterial;

    // 工作台碰撞箱
    public BoxCollider WorkbenchCollider;

    // 工件碰撞箱（动态加载后填充）
    public List<BoxCollider> WorkpieceColliders = new();

    private GlbLoader glbLoader;

    // 碰撞箱可视化相关
    private List<int> colliderVisualIds = new();
    private ColliderVisualizer colliderVisualizer;

    void Awake()
    {
        glbLoader = GetComponent<GlbLoader>();
        WorkbenchCollider = GetComponent<BoxCollider>();
    }

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void Bind(WeldTask task)
    {
        if (task == null) return;
        // 设置工作台位置
        Vector3 originPos = MathUtil.D2UPosition(task.UserOrigin);
        originPos.y -= 0.5f;
        transform.position = originPos;
    }

    public async Task LoadWorkpiece(Workpiece w, string directory)
    {
        // 加载工件到工作台上
        if (w == null || glbLoader == null) return;
        string file = Path.Combine(directory, w.FileName);
        if (File.Exists(file))
        {
            Workpiece = await glbLoader.LoadAsync(file, Origin);
            Vector3 pos = MathUtil.D2UPosition(w.Position);
            Quaternion rot = MathUtil.D2URotation(Quaternion.Euler(w.Rotation));
            Vector3 scale = MathUtil.Abs(MathUtil.D2UPosition(w.Scale));
            Workpiece.transform.SetLocalPositionAndRotation(pos, rot);
            Workpiece.transform.localScale = w.Scale;

            // 清空旧的工件碰撞箱引用
            WorkpieceColliders.Clear();

            foreach (var c in w.Colliders)
            {
                BoxCollider box = Workpiece.AddComponent<BoxCollider>();

                // 1. 先把 Data 坐标系的点转换到 Unity 坐标系
                Vector3 startU = MathUtil.D2UPosition(c.Min);
                Vector3 endU = MathUtil.D2UPosition(c.Max);

                // 2. 变换到工件的局部空间（逆向应用工件的旋转）
                Vector3 startLocal = Quaternion.Inverse(rot) * startU;
                Vector3 endLocal = Quaternion.Inverse(rot) * endU;

                // 3. 应用缩放
                startLocal = MathUtil.Division(startLocal, scale);
                endLocal = MathUtil.Division(endLocal, scale);

                // 4. 计算中心点和尺寸
                box.center = (startLocal + endLocal) / 2f;
                box.size = MathUtil.Abs(startLocal - endLocal);

                // 保存引用
                WorkpieceColliders.Add(box);
            }
        }
        else
        {
            Debug.LogWarning($"Workpiece file does not exist: {file}");
        }
    }

    public Vector3 GetOriginPoint()
    {
        // 获取工作台原点位置（数据坐标系）
        return Origin != null ? MathUtil.U2DPosition(Origin.position) : Vector3.zero;
    }

    /// <summary>
    /// 添加所有碰撞箱到可视化器（工作台 1 个 + 工件若干个）
    /// </summary>
    public void AddCollidersToVisualizer(ColliderVisualizer visualizer)
    {
        if (visualizer == null) return;

        colliderVisualizer = visualizer;
        colliderVisualIds.Clear();

        // 工作台碰撞箱（青色，静态）
        if (WorkbenchCollider != null)
        {
            int id = visualizer.Add(WorkbenchCollider, Color.cyan);
            colliderVisualIds.Add(id);
        }

        // 工件碰撞箱（绿色，静态）
        Color workpieceColor = Color.green;
        foreach (var collider in WorkpieceColliders)
        {
            if (collider == null) continue;
            int id = visualizer.Add(collider, workpieceColor);
            colliderVisualIds.Add(id);
        }
    }

    /// <summary>
    /// 从可视化器中移除所有碰撞箱
    /// </summary>
    public void RemoveCollidersFromVisualizer()
    {
        if (colliderVisualizer == null) return;

        foreach (int id in colliderVisualIds)
        {
            colliderVisualizer.Remove(id);
        }
        colliderVisualIds.Clear();
    }
}
