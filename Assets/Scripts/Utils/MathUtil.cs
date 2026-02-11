using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 数学工具类
/// </summary>
public static class MathUtil
{
    public static Vector3 CalculateCircleCenter(Vector3 A, Vector3 B, Vector3 C)
    {
        // 计算由三点A、B、C确定的圆心
        Vector3 AB = B - A;
        Vector3 AC = C - A;

        Vector3 normal = Vector3.Cross(AB, AC);
        float normalSqrMag = normal.sqrMagnitude;

        if (normalSqrMag < 1e-8f)
        {
            Debug.LogWarning("CalculateCircleCenter: 三点近似共线");
            return A;
        }

        // 垂直平分线求交点公式（向量形式）
        Vector3 toCenter =
            (Vector3.Cross(normal, AB) * AC.sqrMagnitude +
             Vector3.Cross(AC, normal) * AB.sqrMagnitude)
            / (2f * normalSqrMag);

        return A + toCenter;
    }

    public static bool IsAngleBetween(float a, float a0, float a1)
    {
        if (a0 < a1)
            return a > a0 && a < a1;
        else
            return a > a1 && a < a0;
    }

    public static bool IsVector3Close(Vector3 a, Vector3 b)
    {
        return (a - b).sqrMagnitude < 1e-10f;
    }

    public static bool IsQuaternionClose(Quaternion a, Quaternion b)
    {
        return Mathf.Abs(Quaternion.Dot(a, b)) > 0.99999f;
    }

    public static bool IsPoseClose(Pose a, Pose b)
    {
        return IsVector3Close(a.position, b.position) && IsQuaternionClose(a.rotation, b.rotation);
    }

    public static Vector3 EulerZYX(Matrix4x4 m)
    {
        // 按照Z-Y-X顺序从变换矩阵中提取右手系欧拉角
        float sy = Mathf.Sqrt(m.m00 * m.m00 + m.m10 * m.m10);

        bool singular = sy < 1e-6f;

        float x, y, z;
        if (!singular)
        {
            x = Mathf.Atan2(m.m21, m.m22);
            y = Mathf.Atan2(-m.m20, sy);
            z = Mathf.Atan2(m.m10, m.m00);
        }
        else
        {
            x = Mathf.Atan2(-m.m12, m.m11);
            y = Mathf.Atan2(-m.m20, sy);
            z = 0;
        }

        return new Vector3(
            x * Mathf.Rad2Deg,
            y * Mathf.Rad2Deg,
            z * Mathf.Rad2Deg
        );
    }

    /// <summary>
    /// 由角度计算旋转矩阵
    /// </summary>
    public static Matrix4x4 RotX(float deg)
    {
        float r = deg * Mathf.Deg2Rad;
        float c = Mathf.Cos(r);
        float s = Mathf.Sin(r);

        Matrix4x4 m = Matrix4x4.identity;
        m.m11 = c; m.m12 = -s;
        m.m21 = s; m.m22 = c;
        return m;
    }

    public static Matrix4x4 RotY(float deg)
    {
        float r = deg * Mathf.Deg2Rad;
        float c = Mathf.Cos(r);
        float s = Mathf.Sin(r);

        Matrix4x4 m = Matrix4x4.identity;
        m.m00 = c; m.m02 = s;
        m.m20 = -s; m.m22 = c;
        return m;
    }

    public static Matrix4x4 RotZ(float deg)
    {
        float r = deg * Mathf.Deg2Rad;
        float c = Mathf.Cos(r);
        float s = Mathf.Sin(r);

        Matrix4x4 m = Matrix4x4.identity;
        m.m00 = c; m.m01 = -s;
        m.m10 = s; m.m11 = c;
        return m;
    }

    public static Matrix4x4 MDH(float alphaDeg, float a, float thetaDeg, float d)
    {
        // 计算 MDH 变换矩阵
        float alpha = alphaDeg * Mathf.Deg2Rad;
        float theta = thetaDeg * Mathf.Deg2Rad;

        float ca = Mathf.Cos(alpha);
        float sa = Mathf.Sin(alpha);
        float ct = Mathf.Cos(theta);
        float st = Mathf.Sin(theta);

        Matrix4x4 T = Matrix4x4.identity;

        T.m00 = ct;
        T.m01 = -st;
        T.m02 = 0f;
        T.m03 = a;

        T.m10 = st * ca;
        T.m11 = ct * ca;
        T.m12 = -sa;
        T.m13 = -d * sa;

        T.m20 = st * sa;
        T.m21 = ct * sa;
        T.m22 = ca;
        T.m23 = d * ca;

        T.m30 = 0f;
        T.m31 = 0f;
        T.m32 = 0f;
        T.m33 = 1f;

        return T;
    }

    public static Quaternion RotationFromMatrix(Matrix4x4 m)
    {
        // 从变换矩阵提取四元数
        Quaternion q = new();
        float trace = m.m00 + m.m11 + m.m22;

        if (trace > 0f)
        {
            float s = Mathf.Sqrt(trace + 1f) * 2f;
            q.w = 0.25f * s;
            q.x = (m.m21 - m.m12) / s;
            q.y = (m.m02 - m.m20) / s;
            q.z = (m.m10 - m.m01) / s;
        }
        else if (m.m00 > m.m11 && m.m00 > m.m22)
        {
            float s = Mathf.Sqrt(1f + m.m00 - m.m11 - m.m22) * 2f;
            q.w = (m.m21 - m.m12) / s;
            q.x = 0.25f * s;
            q.y = (m.m01 + m.m10) / s;
            q.z = (m.m02 + m.m20) / s;
        }
        else if (m.m11 > m.m22)
        {
            float s = Mathf.Sqrt(1f + m.m11 - m.m00 - m.m22) * 2f;
            q.w = (m.m02 - m.m20) / s;
            q.x = (m.m01 + m.m10) / s;
            q.y = 0.25f * s;
            q.z = (m.m12 + m.m21) / s;
        }
        else
        {
            float s = Mathf.Sqrt(1f + m.m22 - m.m00 - m.m11) * 2f;
            q.w = (m.m10 - m.m01) / s;
            q.x = (m.m02 + m.m20) / s;
            q.y = (m.m12 + m.m21) / s;
            q.z = 0.25f * s;
        }

        return q;
    }

    public static Vector3 NormalizeEulerAngles(Vector3 eulerAngles)
    {
        // 将欧拉角规范到 [-180,180] 范围内
        return new Vector3(
            NormalizeAngle(eulerAngles.x),
            NormalizeAngle(eulerAngles.y),
            NormalizeAngle(eulerAngles.z)
        );
    }

    public static float NormalizeAngle(float angle)
    {
        // 将角度规范到 [-180,180] 范围内
        angle %= 360;
        if (angle < -180)
        {
            angle += 360;
        }
        if (angle > 180)
        {
            angle -= 360;
        }
        return angle;
    }

    // ------------------------
    // 坐标系转换：Unity <-> Data
    // Unity:  forward = +Z, left = -X, up = +Y
    // Data :  forward = +X, left = +Y, up = +Z
    // 位置映射（线性变换）
    // ------------------------
    public static Vector3 UnityToDataPosition(Vector3 u)
    {
        // D.x = U.z, D.y = -U.x, D.z = U.y
        return new Vector3(u.z, -u.x, u.y);
    }

    public static Vector3 DataToUnityPosition(Vector3 d)
    {
        // 逆映射： U.x = -D.y, U.y = D.z, U.z = D.x
        return new Vector3(-d.y, d.z, d.x);
    }

    // ------------------------
    // 旋转映射（通过基向量变换重建旋转）
    // 原理：把 Unity 旋转作用于 Unity 的基向量 (forward, up)，
    // 再把这两个向量映射到 Data 坐标系，用 LookRotation 构建 Data 下的四元数。
    // ------------------------
    public static Quaternion UnityToDataRotation(Quaternion uRot)
    {
        // Unity 基向量经 uRot 变换后的方向（世界向量）
        Vector3 uForward = uRot * Vector3.forward;
        Vector3 uUp = uRot * Vector3.up;

        // 将这两个 world 向量映射到 data 坐标系
        Vector3 dForward = UnityToDataPosition(uForward).normalized;
        Vector3 dUp = UnityToDataPosition(uUp).normalized;

        // 容错：避免零向量导致 LookRotation 报错
        if (dForward.sqrMagnitude < 1e-6f)
            dForward = Vector3.right;
        if (dUp.sqrMagnitude < 1e-6f)
            dUp = Vector3.up;

        return Quaternion.LookRotation(dForward, dUp);
    }

    public static Quaternion DataToUnityRotation(Quaternion dRot)
    {
        Vector3 dForward = dRot * Vector3.forward;
        Vector3 dUp = dRot * Vector3.up;

        Vector3 uForward = DataToUnityPosition(dForward).normalized;
        Vector3 uUp = DataToUnityPosition(dUp).normalized;

        if (uForward.sqrMagnitude < 1e-6f)
            uForward = Vector3.forward;
        if (uUp.sqrMagnitude < 1e-6f)
            uUp = Vector3.up;

        return Quaternion.LookRotation(uForward, uUp);
    }
}
