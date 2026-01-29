using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接IK求解器
/// </summary>
public class WeldIKSolver
{
    public WeldIKResult Solve(Pose tcpPose, RobotModel robot)
    {
        return new WeldIKResult();
    }
}

/// <summary>
/// IK求解结果
/// </summary>
public class WeldIKResult
{
    /// <summary>
    /// 是否有解
    /// </summary>
    public bool Success;

    /// <summary>
    /// 多组可行的解
    /// </summary>
    public List<float[]> Solutions;
}
