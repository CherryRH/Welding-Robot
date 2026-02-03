using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using UnityEngine.Events;
using System.IO;

/// <summary>
/// 仿真环境
/// </summary>
public class SimulationContext : MonoBehaviour
{
    // 模型数据层
    public RobotConfig RobotConfig;
    public RobotModel RobotModel = new();

    // 仿真控制层
    public SimulationStateMachine StateMachine = new();
    public SimulationClock Clock = new(0.01f);

    // 焊接规划层
    public string WeldTaskFileName;
    public WeldTask WeldTask;
    public WeldSeamSampler Sampler = new();
    public WeldPlanner WeldPlanner = new();

    // 路径执行层
    public Trajectory Trajectory = new();

    // 绑定层
    public TransformBinder Binder;
    public WorkpieceBinder WorkpieceBinder;

    // 可视化层
    public WeldSeamVisualizer WeldSeamVisualizer;

    // 绑定事件
    public UnityEvent<SimulationContext> BeforeSimulationUpdate;
    public UnityEvent<SimulationContext> OnSimulationUpdate;
    public UnityEvent<SimulationClock> OnClockUpdate;

    public bool Success = false;

    void Awake()
    {
        // 初始化配置
        RobotModel.Init(RobotConfig);
        FK.Compute(RobotModel);
        Binder.Bind(RobotModel);
        WeldPlanner.Init(WorkpieceBinder.GetOriginPoint());
        // 读取焊接任务文件（WeldTaskFileName 应为绝对路径）
        WeldTask = WeldTaskLoader.LoadFromFile(WeldTaskFileName);
    }

    void Start()
    {
        // 进行仿真准备工作，焊缝采样
        Sampler.Sample(WeldTask);
        // 焊缝可视化
        WeldSeamVisualizer.ShowSeams(Sampler, 1e10f);
        WeldSeamVisualizer.ShowSamplePoints(Sampler, 1e10f);
    }

    void Update()
    {
        if (Clock.Tick(Time.deltaTime))
        {
            // 调用仿真更新前的回调函数
            BeforeSimulationUpdate?.Invoke(this);
            // 时钟更新回调
            if (Clock.IsRunning)
            {
                OnClockUpdate?.Invoke(Clock);
            }
            // 仿真步进
            StateMachine.Update(this, Clock.FixedDeltaTime);

            // 正向运动学计算，更新机械臂姿态、变换矩阵
            FK.Compute(RobotModel);
            // 应用 Unity 机械臂姿态，更新 Unity 坐标和姿态
            Binder.Apply();
            // 调用仿真更新回调函数，更新 UI 等
            OnSimulationUpdate?.Invoke(this);
        }

        // Debug
        if (Input.GetKeyDown(KeyCode.Z))
        {
            Debug.Log(RobotModel.Joints[2].WorldTransform.transpose * Matrix4x4.Rotate(RobotModel.TCPRotation));
        }
    }

    public bool TryChangeState(SimulationState target)
    {
        return StateMachine.TryChangeState(target, this);
    }

    public void TryInput(KeyCode commandKey, int num = -1)
    {
        StateMachine.HandleInput(this, commandKey, num);
    }

    public void TryChangeIKMethod()
    {
        RobotModel.IK.SwitchMethod();
    }
}
