using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// 仿真上下文 — 核心调度器
/// 
/// 生命周期流程：
/// ┌─────────────────────────────────────────────────────────────┐
/// │  Awake()  →  Init()      初始化模型、绑定、规划器           │
/// │  Start()  →  Load()      异步加载工件模型                  │
/// │            →  Build()    优化任务、初始化可视化、碰撞检测    │
/// │                                                             │
/// │  运行时循环（每固定步长）：                                  │
/// │    备份关节 → 状态Update → FK → Apply → 碰撞检测 → 响应     │
/// │                                                             │
/// │  Reset()  →  重置到初始状态（关节归零、时钟归零）           │
/// │  Clear()  →  清空规划数据（路径、轨迹、状态）               │
/// └─────────────────────────────────────────────────────────────┘
/// </summary>
public class SimulationContext : MonoBehaviour
{
    // ============================================================
    // 模型数据层
    // ============================================================
    public RobotConfig RobotConfig;
    public RobotModel RobotModel = new();
    public RobotModel ShadowRobotModel = new();

    // ============================================================
    // 仿真控制层
    // ============================================================
    public SimulationStateMachine StateMachine = new();
    public SimulationClock Clock = new(0.01f);

    // ============================================================
    // 焊接规划层
    // ============================================================
    public string WeldTaskDirectory;
    public string WeldTaskFileName;
    public WeldTask Task;
    public WeldTaskPlanState TaskState = new();
    public TcpPathPlanner TcpPathPlanner = new();

    // ============================================================
    // 轨迹规划层
    // ============================================================
    public TrajectoryPlanner TrajectoryPlanner = new();
    public Trajectory Trajectory = new();

    // ============================================================
    // 碰撞检测层
    // ============================================================
    public CollisionMonitor CollisionMonitor = new();
    public CollisionMonitor ShadowCollisionMonitor = new();

    // ============================================================
    // 绑定层（Unity ↔ 数据模型）
    // ============================================================
    public RobotBinder RobotBinder;
    public RobotBinder ShadowRobotBinder;
    public WorkbenchBinder WorkbenchBinder;
    public EffectBinder EffectBinder;

    // ============================================================
    // 可视化层
    // ============================================================
    public WeldSeamVisualizer WeldSeamVisualizer;
    public TcpPathVisualizer TcpPathVisualizer;
    public ColliderVisualizer ColliderVisualizer;
    public MoltenPoolVisualizer MoltenPoolVisualizer;

    // ============================================================
    // 结果数据
    // ============================================================
    public WeldResultDataWriter ResultWriter = new();

    // ============================================================
    // 事件
    // ============================================================
    public UnityEvent<SimulationContext> BeforeSimulationUpdate;
    public UnityEvent<SimulationContext> OnSimulationUpdate;
    public UnityEvent<SimulationClock> OnClockUpdate;

    public bool Success = false;

    // ============================================================
    // Unity 生命周期
    // ============================================================

    void Awake()
    {
        Init();
    }

    async void Start()
    {
        await Load();
        Build();
    }

    void Update()
    {
        if (Clock.Tick(Time.deltaTime))
        {
            BeforeSimulationUpdate?.Invoke(this);
            if (Clock.IsRunning) OnClockUpdate?.Invoke(Clock);

            // 1. 备份关节角度（用于碰撞回滚）
            StateMachine.BackupJointAngles(this);

            // 2. 状态 Update（可能修改关节角度）
            StateMachine.Update(this, Clock.FixedDeltaTime);

            // 3. 运动学计算 + Transform 更新
            FK.Compute(RobotModel);
            RobotBinder.Apply();

            // 4. 碰撞检测（此时 Transform 已更新）
            CollisionMonitor.Update();

            // 5. 碰撞响应处理
            HandleCollisionResponse();

            // 6. 更新实时数据（速度等）并记录仿真帧
            RobotModel.UpdateRealtimeData(Clock.Time, Clock.FixedDeltaTime);
            if (StateMachine.CurrentState == SimulationState.Work)
            {
                ResultWriter.RecordFrame(Clock.Time, RobotModel, Trajectory.CurrentSegment);
            }

            // 7. 熔池可视化采样（仅焊接段）
            if (Clock.IsRunning && MoltenPoolVisualizer != null && Trajectory.CurrentSegment != null)
            {
                if (Trajectory.CurrentSegment.Type == WeldStateType.Weld)
                {
                    MoltenPoolVisualizer.AddSample(RobotBinder.Tcp, RobotModel.TcpSpeed);
                }
            }

            // 8. 重规划冷却递减
            if (TaskState.ReplanCooldown > 0f)
                TaskState.ReplanCooldown -= Time.deltaTime;

            OnSimulationUpdate?.Invoke(this);
        }

        // Debug
        if (Input.GetKeyDown(KeyCode.Z))
        {

        }
    }

    // ============================================================
    // 碰撞响应
    // ============================================================

    /// <summary>
    /// 碰撞响应处理：回滚、状态切换或重规划
    /// </summary>
    private void HandleCollisionResponse()
    {
        var level = CollisionMonitor.OverallLevel;
        float currentDist = CollisionMonitor.SelfCollision.MinDistance;
        bool isTeleop = StateMachine.CurrentState == SimulationState.Joint ||
                        StateMachine.CurrentState == SimulationState.TCP;

        StateMachine.UpdateLastDistance(currentDist);

        switch (level)
        {
            case CollisionMonitor.CollisionLevel.Safe:
                TaskState.ConsecutiveWarnings = 0;
                TaskState.NarrowSpaceMode = false;
                break;

            case CollisionMonitor.CollisionLevel.Warning:
                // 冷却中：仅累加计数，不触发
                if (TaskState.ReplanCooldown > 0f)
                {
                    TaskState.ConsecutiveWarnings++;
                    if (!TaskState.NarrowSpaceMode &&
                        TaskState.ConsecutiveWarnings >= WeldTaskPlanState.MaxConsecutiveWarnings)
                    {
                        TaskState.NarrowSpaceMode = true;
                        Debug.Log("[Replan] Narrow space detected, allowing continued operation.");
                    }
                    break;
                }

                // 狭窄空间：放行，不重规划
                if (TaskState.NarrowSpaceMode)
                    break;

                // 正常工作状态：触发重规划
                if (StateMachine.CurrentState == SimulationState.Work)
                    TriggerReplan();
                break;

            case CollisionMonitor.CollisionLevel.Collision:
                if (isTeleop)
                {
                    RollbackAndRefresh();
                }
                else if (StateMachine.CurrentState == SimulationState.Work)
                {
                    StateMachine.TryChangeState(SimulationState.Fail, this);
                }
                break;
        }
    }

    /// <summary>
    /// 触发重规划（从当前实际位置重新规划接近路径）
    /// </summary>
    private void TriggerReplan()
    {
        // 必须有当前轨迹段才能重规划
        if (Trajectory.CurrentSegment == null)
        {
            Debug.LogWarning("[Replan] No current segment to replan from.");
            return;
        }

        Debug.Log($"[Replan] Triggered. Current segment: {Trajectory.CurrentSegment.StartPoint.Flag} → {Trajectory.CurrentSegment.EndPoint.Flag}");

        // 标记重规划中
        TaskState.IsReplanning = true;
        TaskState.ReplanCooldown = WeldTaskPlanState.ReplanCooldownDuration;
        TaskState.ConsecutiveWarnings = 0;

        // 重规划 TCP 路径
        TcpPathPlanner.ReplanFromPosition(
            Trajectory.CurrentSegment,
            ShadowCollisionMonitor,
            ShadowRobotBinder);

        // 清空轨迹缓冲区（下一帧 WorkState 会自动重建）
        Trajectory.Clear();

        // 刷新路径可视化
        TcpPathVisualizer.ShowTcpPathPoints(TcpPathPlanner);

        TaskState.IsReplanning = false;
    }

    /// <summary>
    /// 回滚并刷新姿态
    /// </summary>
    private void RollbackAndRefresh()
    {
        StateMachine.RollbackLastMove(this);
        FK.Compute(RobotModel);
        RobotBinder.Apply();
        CollisionMonitor.Update();
    }

    // ============================================================
    // 初始化 / 加载 / 构建 / 重置 / 清空
    // ============================================================

    /// <summary>
    /// 初始化：绑定模型、规划器，加载任务数据
    /// 在 Awake() 中调用，也可在外部调用以重新初始化
    /// </summary>
    public void Init()
    {
        // 加载焊接任务数据
        string weldTaskFile = Path.Combine(WeldTaskDirectory, WeldTaskFileName);
        WeldTaskData data = WeldTaskDataLoader.LoadFromFile(weldTaskFile);
        Task = new(data);

        // 初始化机器人模型
        RobotModel.Init(RobotConfig);
        ShadowRobotModel.Init(RobotConfig);

        // 计算初始运动学
        FK.Compute(RobotModel);
        FK.Compute(ShadowRobotModel);

        // 绑定到 Unity
        RobotBinder.Bind(RobotModel);
        ShadowRobotBinder.Bind(ShadowRobotModel);
        WorkbenchBinder.Bind(Task);

        // 设置用户坐标系
        RobotModel.SetUserOffset(WorkbenchBinder.GetOriginPoint());
        ShadowRobotModel.SetUserOffset(WorkbenchBinder.GetOriginPoint());

        // 初始化规划器
        TcpPathPlanner.Init(RobotModel, TaskState);
        TrajectoryPlanner.Init(RobotModel, Trajectory);
    }

    /// <summary>
    /// 异步加载：加载工件模型
    /// 在 Start() 中调用
    /// </summary>
    public async Task Load()
    {
        await WorkbenchBinder.LoadWorkpiece(Task.Workpiece, WeldTaskDirectory);
    }

    /// <summary>
    /// 构建：优化任务、初始化可视化和碰撞检测
    /// 在 Load() 之后调用
    /// </summary>
    public void Build()
    {
        // 优化焊缝顺序
        Task.Optimize();

        // 显示焊缝可视化
        WeldSeamVisualizer.ShowSeams(Task, 1e10f);

        // 初始化碰撞可视化
        ShowColliders();

        // 初始化碰撞检测器
        CollisionMonitor.Init(RobotBinder, WorkbenchBinder);
        ShadowCollisionMonitor.Init(ShadowRobotBinder, WorkbenchBinder);
    }

    /// <summary>
    /// 重置：将仿真恢复到初始状态
    /// - 关节角度归零
    /// - 时钟归零
    /// - 刷新运动学和 Transform
    /// - 清空规划数据
    /// 
    /// 在进入 Idle 状态时调用，确保每次开始都是干净状态
    /// </summary>
    public void Reset()
    {
        // 关节角度归零
        RobotModel.SetJointAngles(new float[RobotModel.JointsCount]);
        ShadowRobotModel.SetJointAngles(new float[ShadowRobotModel.JointsCount]);

        // 时钟归零
        Clock.Reset();

        // 重置实时数据状态
        RobotModel.ResetRealtimeData();

        // 刷新运动学和 Transform
        FK.Compute(RobotModel);
        RobotBinder.Apply();
        ShadowRobotBinder.Apply();

        // 清空规划数据
        Clear();

        // 刷新 UI
        OnClockUpdate?.Invoke(Clock);
    }

    /// <summary>
    /// 清空：清除规划相关的临时数据
    /// - TCP 路径规划器
    /// - 轨迹
    /// - 任务规划状态
    /// - TCP 路径可视化
    /// </summary>
    public void Clear()
    {
        TcpPathPlanner.Clear();
        Trajectory.Clear();
        TaskState.Reset();
        if (TcpPathVisualizer != null) TcpPathVisualizer.Clear();
        if (MoltenPoolVisualizer != null) MoltenPoolVisualizer.Clear();
    }

    // ============================================================
    // 碰撞可视化
    // ============================================================

    public void ShowColliders()
    {
        if (ColliderVisualizer == null) return;
        WorkbenchBinder.AddCollidersToVisualizer(ColliderVisualizer);
        RobotBinder.AddCollidersToVisualizer(ColliderVisualizer);
    }

    public void HideColliders()
    {
        if (ColliderVisualizer == null) return;
        WorkbenchBinder.RemoveCollidersFromVisualizer();
        RobotBinder.RemoveCollidersFromVisualizer();
    }

    // ============================================================
    // 公共接口
    // ============================================================

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
