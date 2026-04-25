using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接任务
/// </summary>
public class WeldTask
{
    /// <summary>
    /// 任务名称
    /// </summary>
    public string TaskName;

    /// <summary>
    /// 任务文件路径
    /// </summary>
    public string TaskFilePath;

    /// <summary>
    /// 用户坐标系原点（米）
    /// </summary>
    public Vector3 UserOrigin;

    /// <summary>
    /// 工件
    /// </summary>
    public Workpiece Workpiece;

    /// <summary>
    /// 焊缝列表（Optimize 后会插入包角圆弧段）
    /// </summary>
    public List<WeldSeam> WeldSeams = new();

    /// <summary>
    /// 当前最大焊缝 ID（正整数），用于生成新包角圆弧的 ID
    /// </summary>
    private int _nextId;

    public WeldTask(WeldTaskData data, string taskFilePath)
    {
        TaskFilePath = taskFilePath;
        TaskName = data.TaskName;
        UserOrigin = data.UserOrigin;

        if (data.Workpiece != null)
            Workpiece = new Workpiece(data.Workpiece);

        if (data.WeldSeams != null)
        {
            foreach (var seamData in data.WeldSeams)
            {
                // 记录最大 ID，确保新生成的包角弧不冲突
                if (seamData.ID > _nextId) _nextId = seamData.ID;

                WeldSeam seam = seamData.Type switch
                {
                    WeldSeamData.WeldSeamType.Line => new LineSeam(seamData),
                    WeldSeamData.WeldSeamType.Arc  => new ArcSeam(seamData),
                    _ => null
                };
                if (seam != null)
                    WeldSeams.Add(seam);
            }
        }

        // ID 从最大 ID + 1 开始
        _nextId++;
    }

    // ==================== 优化项 ====================

    /// <summary>
    /// 在相邻焊缝之间插入包角圆弧过渡段，支持多环检测
    /// </summary>
    private void InsertCornerArcs()
    {
        if (WeldSeams == null || WeldSeams.Count < 2) return;

        // 维护当前连续焊缝串的起点索引
        int chainStart = 0;

        int i = 0;
        while (i < WeldSeams.Count - 1)
        {
            WeldSeam current = WeldSeams[i];
            WeldSeam next    = WeldSeams[i + 1];

            // 当前焊缝的 CornerRadius 控制包角；0 或负值表示不做
            float r = current.CornerRadius;
            if (r <= 0f)
            {
                // 当前焊缝不做包角，检查当前串是否成环，然后开启新串
                TryCloseLoop(chainStart, i);
                chainStart = i + 1;
                i++;
                continue;
            }

            // 端点不相邻 → 断开，检查当前串是否成环，然后开启新串
            if (!MathUtil.IsVector3Equal(current.EndPoint, next.StartPoint))
            {
                TryCloseLoop(chainStart, i);
                chainStart = i + 1;
                i++;
                continue;
            }

            // 构造包角圆弧
            if (!TryCreateCornerArc(current, next, r, out WeldSeam corner))
            {
                // 构造失败，检查当前串是否成环，然后开启新串
                TryCloseLoop(chainStart, i);
                chainStart = i + 1;
                i++;
                continue;
            }

            // 缩短相邻焊缝端点，插入圆弧段
            TrimAndInsert(i, corner);
            i += 2; // 跳过刚插入的圆弧段
        }

        // 最后检查末尾的串是否成环
        TryCloseLoop(chainStart, WeldSeams.Count - 1);
    }

    /// <summary>
    /// 尝试闭合从 chainStart 到 chainEnd 的焊缝串
    /// 如果该串的起点和终点相连，则插入闭环包角
    /// </summary>
    private void TryCloseLoop(int chainStart, int chainEnd)
    {
        // 至少需要两条焊缝才能成环
        if (chainEnd - chainStart < 1) return;

        WeldSeam first = WeldSeams[chainStart];
        WeldSeam last  = WeldSeams[chainEnd];

        // 使用最后一条焊缝的 CornerRadius
        float r = last.CornerRadius;
        if (r <= 0f) return;

        // 检查首尾是否相连
        if (!MathUtil.IsVector3Equal(last.EndPoint, first.StartPoint))
            return;

        // 构造闭环包角圆弧
        if (!TryCreateCornerArc(last, first, r, out WeldSeam corner))
            return;

        // 缩短首尾焊缝端点
        if (last is LineSeam ls)
            ls.SetEndPoint(corner.StartPoint);
        if (first is LineSeam fs)
            fs.SetStartPoint(corner.EndPoint);

        // 在 chainEnd 之后插入圆弧段
        WeldSeams.Insert(chainEnd + 1, corner);
    }

    /// <summary>
    /// 尝试在两条相邻焊缝之间构造包角圆弧（复用 ArcSeam）
    /// </summary>
    private bool TryCreateCornerArc(
        WeldSeam seamA, WeldSeam seamB,
        float radius,
        out WeldSeam arc)
    {
        arc = null;

        Vector3 V  = seamA.EndPoint; // ≈ seamB.StartPoint
        Vector3 d1 = seamA.GetTangent(1f); // A 沿走向离开
        Vector3 d2 = seamB.GetTangent(0f); // B 沿走向进入

        // 计算切点 T1、T2、圆心 O、弧中点 M
        if (!MathUtil.FilletArc(V, d1, d2, radius,
            out Vector3 T1, out Vector3 T2, out Vector3 O, out Vector3 M))
            return false;

        // 切点必须在各自焊缝的有效范围内
        if (!seamA.ContainsPoint(T1) || !seamB.ContainsPoint(T2))
            return false;

        // 用当前焊缝（LineSeam）构造 ArcSeam
        if (seamA is not LineSeam line)
            return false;

        int id = _nextId++;
        arc = new ArcSeam(line, id, radius, T1, T2, O, M);
        return true;
    }

    /// <summary>
    /// 将 corner 圆弧插入焊缝列表第 i+1 位置，
    /// 并将 seam[i] 的终点和 seam[i+1] 的起点截断到 corner 的两端
    /// </summary>
    private void TrimAndInsert(int index, WeldSeam corner)
    {
        if (WeldSeams[index] is LineSeam ls)
            ls.SetEndPoint(corner.StartPoint);

        if (WeldSeams[index + 1] is LineSeam ns)
            ns.SetStartPoint(corner.EndPoint);

        WeldSeams.Insert(index + 1, corner);
    }

    // ==================== 公共入口 ====================

    /// <summary>
    /// 优化焊缝执行顺序及形状。依次调用各优化项。
    /// </summary>
    public void Optimize()
    {
        InsertCornerArcs();
    }
}