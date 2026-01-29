using UnityEngine;

/// <summary>
/// 焊缝几何抽象类
/// </summary>
public abstract class WeldSeamGeometry
{
    /// <summary>
    /// 按弧长参数 s（0~length）取点
    /// </summary>
    public abstract Vector3 GetPoint(float s);

    /// <summary>
    /// 取切线方向（单位向量）
    /// </summary>
    public abstract Vector3 GetTangent(float s);

    /// <summary>
    /// 几何总长度
    /// </summary>
    public abstract float Length { get; }

    /// <summary>
    /// 几何所在平面的法向（Line 可返回固定值）
    /// </summary>
    public abstract Vector3 Normal { get; }
}

public class LineGeometry : WeldSeamGeometry
{
    public Vector3 Start;
    public Vector3 End;

    private Vector3 direction;
    private float length;
    private Vector3 normal;

    public LineGeometry(Vector3 start, Vector3 end, Vector3 normal)
    {
        Start = start;
        End = end;
        this.normal = normal;

        direction = (End - Start).normalized;
        length = Vector3.Distance(Start, End);
    }

    public override float Length => length;

    public override Vector3 Normal => normal;

    public override Vector3 GetPoint(float s)
    {
        return Start + direction * Mathf.Clamp(s, 0, length);
    }

    public override Vector3 GetTangent(float s)
    {
        return direction;
    }
}

public class ArcGeometry : WeldSeamGeometry
{
    public Vector3 Center;
    public float Radius;

    public float StartAngle;   // 弧参数
    public float DeltaAngle;   // 带符号，决定顺逆
    private float length;
    private Vector3 normal;

    private Vector3 u; // 圆平面基向量
    private Vector3 v;

    public ArcGeometry(Vector3 start, Vector3 middle, Vector3 end, Vector3 normal)
    {
        Center = MathUtil.CalculateCircleCenter(start, middle, end);
        Radius = Vector3.Distance(start, Center);

        Vector3 v1 = (start - Center).normalized;
        Vector3 v2 = (end - Center).normalized;

        this.normal = Vector3.Cross((middle - start), (end - middle)).normalized;

        if (this.normal.sqrMagnitude < 1e-6f)
            this.normal = Vector3.forward;

        // 构建圆平面坐标系
        u = v1;
        v = Vector3.Cross(this.normal, u).normalized;

        StartAngle = 0f;

        float rawAngle = Mathf.Atan2(
            Vector3.Dot(v2, v),
            Vector3.Dot(v2, u)
        );

        // 用 middle 点判断取哪一段弧
        float midAngle = Mathf.Atan2(
            Vector3.Dot((middle - Center).normalized, v),
            Vector3.Dot((middle - Center).normalized, u)
        );

        // 确保中间点落在弧段上
        if (!IsAngleBetween(midAngle, 0f, rawAngle))
        {
            rawAngle = rawAngle > 0
                ? rawAngle - 2 * Mathf.PI
                : rawAngle + 2 * Mathf.PI;
        }

        DeltaAngle = rawAngle;
        length = Mathf.Abs(Radius * DeltaAngle);
    }

    public override float Length => length;
    public override Vector3 Normal => normal;

    public override Vector3 GetPoint(float s)
    {
        float t = Mathf.Clamp01(s / length);
        float angle = StartAngle + DeltaAngle * t;

        return Center + Radius * (Mathf.Cos(angle) * u + Mathf.Sin(angle) * v);
    }

    public override Vector3 GetTangent(float s)
    {
        float t = Mathf.Clamp01(s / length);
        float angle = StartAngle + DeltaAngle * t;

        Vector3 tangent =
            -Mathf.Sin(angle) * u +
             Mathf.Cos(angle) * v;

        if (DeltaAngle < 0)
            tangent = -tangent;

        return tangent.normalized;
    }

    private bool IsAngleBetween(float a, float a0, float a1)
    {
        if (a0 < a1)
            return a > a0 && a < a1;
        else
            return a > a1 && a < a0;
    }
}
