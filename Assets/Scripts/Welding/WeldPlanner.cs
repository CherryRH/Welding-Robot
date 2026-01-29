using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 焊接规划器
/// </summary>
public class WeldPlanner
{
    private WeldIKSolver ikSolver = new();

    public Vector3 UserCoordinateSystem;

    /// <summary>
    /// 焊接工作参数
    /// </summary>
    public float SafetyHeight = 0.2f;

    public void Init(Vector3 ucs) {
        // 初始化用户坐标系为工件原点位置
        UserCoordinateSystem = ucs;
    }

    public void Plan(WeldTask task, RobotModel robot, Trajectory trajectory) {
        
    }
}
