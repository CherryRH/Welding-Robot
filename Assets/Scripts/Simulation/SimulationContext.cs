using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

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
    public WeldTask Task;
    public WeldTaskPlanState TaskState = new();
    public TcpPathPlanner TcpPathPlanner = new();

    // 路径规划层
    public TrajectoryPlanner TrajectoryPlanner = new();
    public Trajectory Trajectory = new();

    // 绑定层
    public TransformBinder Binder;
    public WorkpieceBinder WorkpieceBinder;

    // 可视化层
    public WeldSeamVisualizer WeldSeamVisualizer;
    public TcpPathVisualizer TcpPathVisualizer;

    // 绑定事件
    public UnityEvent<SimulationContext> BeforeSimulationUpdate;
    public UnityEvent<SimulationContext> OnSimulationUpdate;
    public UnityEvent<SimulationClock> OnClockUpdate;

    public bool Success = false;

    void Awake()
    {
        Init();
    }

    void Start()
    {
        Build();
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

    private void Init()
    {
        // 初始化配置
        RobotModel.Init(RobotConfig);
        RobotModel.SetUserOffset(WorkpieceBinder.GetOriginPoint());
        FK.Compute(RobotModel);
        Binder.Bind(RobotModel);
        TcpPathPlanner.Init(RobotModel, TaskState);
        TrajectoryPlanner.Init(RobotModel, Trajectory);
        // 读取焊接任务文件（WeldTaskFileName 应为绝对路径）
        WeldTaskData data = WeldTaskDataLoader.LoadFromFile(WeldTaskFileName);
        // 构建焊缝任务
        Task = new(data);
    }

    public void Build()
    {
        // 优化焊缝任务
        Task.Optimize();
        // 可视化焊缝
        WeldSeamVisualizer.ShowSeams(Task, 1e10f);
    }

    public void Clear()
    {
        // 清除规划数据
        TcpPathPlanner.Clear();
        Trajectory.Clear();
        TaskState.Reset();
        // 清除可视化
        TcpPathVisualizer.Clear();
    }
}