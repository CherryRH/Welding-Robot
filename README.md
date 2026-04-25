# Welding Robot — 焊接机器人离线仿真系统

> Unity 2022 LTS | 基于 MDH 参数模型的 6 轴工业机器人仿真

---

## 项目概述

本项目是一个 **Unity 焊接机器人离线仿真系统**，模拟 FANUC M10-iD 等 6 轴工业机器人在工件上执行焊接任务的全流程。系统以 Data 右手坐标系（forward=+X, left=+Y, up=+Z）为数据核心，通过 Unity 场景实现可视化。

**核心功能**：
- 完整的正/逆运动学（FK/IK）求解
- TCP 空间路径规划（接近 → 焊接 → 离开）
- 关节空间轨迹规划（速度限制检查）
- 实时碰撞检测与重规划
- 路径后处理平滑优化（PathSmoother）
- 焊接熔池可视化
- 仿真过程回放（Replay）

---

## 架构总览

```
App
├── Managers          # 应用管理器
│   ├── AppManager           # 多仿真实例切换
│   └── KeyInputManager      # 键盘输入管理
└── UI                # 界面
    ├── AppView              # 应用视图
    ├── ControlView          # 控制面板视图
    └── SimulationView       # 仿真视图

Simulation
├── SimulationContext        # 仿真上下文（核心调度器）
├── SimulationClock          # 仿真时钟（固定步长 10ms）
├── SimulationStateMachine   # 状态机
└── States                   # 状态实现
    ├── IdleState            # 空闲
    ├── WorkState            # 执行焊接任务
    ├── SucceedState         # 任务成功
    ├── FailState            # 任务失败
    ├── ReplayState          # 仿真回放
    ├── JointState           # 关节遥操作
    └── TCPState             # TCP 遥操作

Robot
├── Model                   # 机器人模型
│   ├── RobotModel          # 机器人模型（关节状态、TCP、运动学）
│   └── JointModel          # 单关节模型
└── Kinematics              # 运动学
    ├── FK                  # 正向运动学（MDH 变换）
    ├── IK                  # 逆向运动学（CCD / JT / 解析法）
    ├── IKResult            # IK 结果（多解支持）
    └── JointParameters     # 关节 MDH 参数

Welding
├── Data                    # 数据层
│   ├── WeldTaskData        # 焊接任务数据
│   ├── WeldSeamData        # 焊缝数据
│   ├── WorkpieceData       # 工件数据
│   ├── WeldResultData      # 焊接结果数据
│   ├── WeldResultFrame     # 单帧数据
│   ├── WeldResultDataWriter # 结果导出器
│   └── WeldTaskDataLoader  # JSON 文件加载器
├── Object                  # 焊接对象
│   ├── WeldSeam            # 焊缝基类
│   ├── LineSeam            # 直线焊缝
│   ├── ArcSeam             # 弧线焊缝
│   └── Workpiece           # 工件
├── Path                    # TCP 路径规划
│   ├── TcpPathPlanner      # TCP 路径调度器
│   ├── TcpPathPoint        # TCP 路径点
│   ├── ApproachPathPlanner # 接近路径规划
│   ├── WeldPathPlanner     # 焊接路径规划
│   ├── AdjustPathPlanner   # 调整路径规划（重规划时使用）
│   ├── RrtPathPlanner      # RRT 避障路径规划
│   └── PathSmoother        # 路径后处理平滑器
├── Task                    # 任务层
│   ├── WeldTask            # 焊接任务
│   ├── WeldTaskPlanState   # 任务规划状态
│   └── WeldStateType       # 焊接状态枚举
└── Trajectory              # 关节空间轨迹
    ├── Trajectory           # 轨迹（SortedDictionary 存储）
    ├── TrajectorySegment    # 轨迹段
    ├── TrajectoryPlanner    # 轨迹规划器
    └── IJointInterpolator   # 关节插值接口
        ├── LinearJointInterpolator          # 线性插值
        └── CubicHermiteSegmentInterpolator  # 三次 Hermite 样条插值

Collision
└── CollisionMonitor        # 碰撞监测器
    ├── BodyCollision       # 机械臂本体 vs 环境
    ├── GunCollision        # 焊枪 vs 环境
    └── SelfCollision       # 机械臂自体碰撞

DataFlow                   # 数据绑定层（Unity ↔ 数据模型）
├── RobotBinder              # 机械臂绑定（关节角度 → Transform）
├── WorkbenchBinder          # 工作台/工件绑定
└── EffectBinder             # 焊接特效绑定

Visualization              # 可视化层
├── ColliderVisualizer       # 碰撞箱可视化（GL 渲染）
├── TcpPathVisualizer        # TCP 路径点可视化
├── WeldSeamVisualizer       # 焊缝可视化
└── MoltenPoolVisualizer     # 熔池可视化

Utils                        # 工具类
├── MathUtil                 # 数学工具（坐标转换、MDH、欧拉角）
├── JsonUtil                 # JSON 序列化
├── LogUtil                  # 日志工具
└── GlbLoader                # GLB 模型加载器

Config                       # 配置
└── RobotConfig              # 机器人配置（SO）
```

---

## 坐标系约定

| 坐标系 | 定义 | 用途 |
|--------|------|------|
| **Data 坐标系** | forward=+X, left=+Y, up=+Z | 所有数据文件、内部计算 |
| **Unity 坐标系** | forward=+Z, left=-X, up=+Y | 场景渲染、Transform |

转换方法见 `MathUtil.cs`：
- `D2UPosition / U2DPosition` — 位置
- `D2URotation / U2DRotation` — 旋转（通过基向量映射）

---

## 仿真状态机

```
         ┌─── Space   ┌───
   ┌────►│  Idle  │◄────┐
   │     └───┬────┘     │
   │         │ Space    │ 轨迹执行完毕
   │         ▼          │
   │    ┌─────────┐     │
   │    │  Work   │─────┘
   │    └────┬────┘
   │         │ 失败/重规划超限
   │         ▼
   │    ┌─────────┐     成功
   │    │  Fail   │◄───────────
   │    └────┬────┘
   │         │ Tab
   │         ▼
   │    ┌─────────┐
   │    │ Succeed │◄───────────
   │    └────┬────┘
   │         │ Tab
   │         ▼
   │    ┌─────────┐
   └────│ Replay  │  回放已记录的仿真帧
        └─────────┘

   Shift + 1-6 ──► JointState（关节遥操作）
   Shift + 7    ──► TCPState（TCP 遥操作）
```

**状态说明**：
- `Idle` — 空闲，可按 Space 开始
- `Work` — 执行焊接任务（规划 → 执行 → 碰撞监测 → 重规划）
- `Succeed` — 任务成功
- `Fail` — 任务失败（重规划超限/碰撞/IK 无解）
- `Replay` — 回放已记录的仿真帧数据
- `Joint` — 关节遥操作模式
- `TCP` — TCP 遥操作模式

---

## 运动学

### 正向运动学（FK）
- 基于 **MDH 参数**（Modified Denavit-Hartenberg）
- 变换顺序：θ → d → a → α
- 支持 Tool Rotation 偏姿

### 逆向运动学（IK）
三种算法可切换（按 Shift）：

| 算法 | 类型 | 特点 |
|------|------|------|
| `ANALYTIC` | 解析法 | 速度快，有多解（肘上/肘下、翻转型） |
| `CCD` | 循环坐标下降 | 迭代收敛，适合连续路径 |
| `JT` | Jacobian 转置 | 数值稳定，适合精细控制 |

---

## 路径规划

### 规划层次
```
焊接任务 → TCP路径规划 → 轨迹规划 → 关节运动
 (焊缝)   (TcpPathPlanner) (TrajectoryPlanner) (FK)
```

### TCP 路径规划
`TcpPathPlanner` 负责调度三种子规划器：

| 规划器 | 功能 | 输出 |
|--------|------|------|
| `ApproachPathPlanner` | 从当前位置到焊缝起点 | 接近路径点 |
| `WeldPathPlanner` | 沿焊缝移动 | 焊接路径点 |
| `AdjustPathPlanner` | 调整路径（速度超限/奇异点） | 调整路径点 |

### 接近路径策略
`ApproachPathPlanner` 支持三种策略：

| 策略 | 说明 |
|------|------|
| `Direct` | 直线插值（默认，最快） |
| `RRT` | 快速随机树避障路径 |
| `Safe` | 经安全高度绕行 |

### 路径平滑优化
`PathSmoother` 对 RRT 等算法生成的路径进行后处理优化：

**四种力模型**：
1. **引力（Attraction）**：目标点吸引，引导路径向终点收缩
2. **拉直力（Pull）**：向起终点垂直平面靠拢，减少绕路
3. **平滑力（Laplacian）**：向相邻节点中点移动，消除抖动
4. **排斥力（Repulsion）**：障碍物排斥，保持安全距离

每次移动后进行 IK + 碰撞检验，保证路径有效性。

---

## 轨迹规划

### 数据结构
- **TcpPathPlanner**：`LinkedList<TcpPathPoint>` 管理 TCP 路径点
- **Trajectory**：`SortedDictionary<float, TrajectorySegment>` 管理轨迹段，按时间戳存储

### 轨迹插值
- **线性插值**：恒定关节速度
- **三次 Hermite 样条**：平滑起止速度，支持单调速度限制（Fritsch–Carlson）

### 关键特性
- **速度限制检查**：规划时验证关节角速度是否超限
- **重规划机制**：碰撞 Warning → 触发 RRT 重规划
- **奇异点处理**：J5=0° 时进入奇异姿态，自动归零后离开
- **腹部翻转**：允许腕部 180° 翻转以获得更优姿态

---

## 碰撞检测系统

### 碰撞分组

| 组 | 检测对象 | 用途 |
|----|----------|------|
| `Body` | J0~J6（不含焊枪）vs 环境 | 机械臂本体碰撞 |
| `Gun` | 焊枪 vs 环境 | 焊枪碰撞（阈值更小） |
| `Self` | J4/J5/J6+焊枪 vs J0/J1/J2 | 自体碰撞 |

### 碰撞等级

| 等级 | 说明 |
|------|------|
| `Safe` | 距离 > Warning 阈值 |
| `Warning` | 距离 ≤ Warning 阈值，触发重规划 |
| `Collision` | 距离 ≤ Collision 阈值（或穿透），任务失败 |

### 颜色约定
| 碰撞箱类型 | 颜色 |
|-----------|------|
| 机械臂 | 黄色 |
| 工作台 | 青色 |
| 工件 | 绿色 |

---

## 熔池可视化

`MoltenPoolVisualizer` 基于射线检测渲染焊道条带：

**原理**：
- 每帧从 TCP 两侧发射射线，检测工件表面交点
- 用交点对构造三角条带 Mesh
- 条带宽度与 TCP 线速度成反比（速度越快焊道越窄）

**生命周期**：
- `StartStrip()` — 焊接段开始
- `AddSample(tcp, speed, segmentType)` — 每帧采样
- `FinalizeStrip()` — 焊接段结束

支持 Work 和 Replay 两种状态。

---

## 数据导出

### WeldResultData 结构

```csharp
public class WeldResultData
{
    public string TaskName;              // 任务名称
    public PlanStatus PlanStatus;        // 规划状态
    public List<WeldPointData> Points;   // 规划路径点
    public SortedDictionary<float, WeldResultFrame> Frames; // 仿真帧数据
}
```

### WeldResultFrame 结构

```csharp
public class WeldResultFrame
{
    public float Timestamp;              // 时间戳
    public Vector3 TcpPosition;          // TCP 位置
    public float[] JointAngles;          // 关节角度
    public float TcpSpeed;               // TCP 线速度
    public float[] JointVelocities;      // 关节角速度
    public float[] JointAccelerations;   // 关节角加速度
    public WeldStateType SegmentType;    // 段类型（Approach/Weld）
    public int SeamId;                   // 焊缝 ID
}
```

---

## 配置说明

### RobotConfig（ScriptableObject）
- MDH 参数（Alpha, A, Theta, D）
- 关节角度限制（Min, Max）及最大角速度
- TCP 偏移、法兰偏移、Tool Rotation
- 遥操作速度参数

### 焊接任务文件（JSON）
路径：`Welding Tasks/*.json`

```json
{
  "TaskName": "任务名",
  "UserOrigin": { "x": 0, "y": 0, "z": 0 },
  "Workpiece": {
    "FileName": "模型.glb",
    "Position": { "x": -0.005, "y": -0.25, "z": 0.0 },
    "Rotation": { "x": 0, "y": 0, "z": -90 },
    "Scale": { "x": 0.5, "y": 0.5, "z": 0.5 },
    "Colliders": [{ "Start": {}, "End": {} }]
  },
  "WeldSeams": [
    {
      "Type": "Line",
      "Name": "Seam_1",
      "Speed": 0.05,
      "GunAngle": 45,
      "GunDistance": 0.01,
      "StartPoint": { "x": -0.2, "y": 0, "z": 0 },
      "EndPoint": { "x": -0.1, "y": 0, "z": 0 }
    }
  ]
}
```

---

## 关键文件索引

| 功能 | 文件 |
|------|------|
| 应用入口 | `AppManager.cs` |
| 仿真核心 | `SimulationContext.cs` |
| 状态机 | `SimulationStateMachine.cs` + `States/*.cs` |
| 仿真时钟 | `SimulationClock.cs` |
| 正向运动学 | `FK.cs` |
| 逆向运动学 | `IK.cs` |
| TCP 路径规划 | `TcpPathPlanner.cs` |
| 接近路径规划 | `ApproachPathPlanner.cs` |
| RRT 避障 | `RrtPathPlanner.cs` |
| 路径平滑 | `PathSmoother.cs` |
| 轨迹规划 | `TrajectoryPlanner.cs` |
| 轨迹执行 | `Trajectory.cs` |
| 碰撞检测 | `CollisionMonitor.cs` |
| 熔池可视化 | `MoltenPoolVisualizer.cs` |
| 坐标转换 | `MathUtil.cs` |
| 工件加载 | `WorkbenchBinder.cs` |
| 机械臂绑定 | `RobotBinder.cs` |
| 结果导出 | `WeldResultDataWriter.cs` |

---

## 技术栈

- **引擎**：Unity 2022 LTS
- **语言**：C# (.NET Standard 2.1)
- **模型格式**：GLB（通过 GlbLoader）
- **数据格式**：JSON

---

## 开发状态

本项目为毕业设计项目，主要功能已实现完成，进入论文撰写阶段。

**已完成**：
- [x] 正/逆运动学（FK/IK）
- [x] TCP 路径规划（接近/焊接/调整）
- [x] 轨迹规划（线性/Hermite 插值）
- [x] 碰撞检测（Body/Gun/Self 三组）
- [x] 实时重规划机制
- [x] RRT 避障路径规划
- [x] PathSmoother 路径后处理
- [x] 熔池可视化
- [x] 仿真回放（Replay）
- [x] 数据导出

---

## 参考文献

1. Siciliano, B., et al. "Robotics: Modelling, Planning and Control"
2. FANUC M10-iD 参数手册
3. RRT-Connect 算法 (Kuffner & LaValle, 2000)
4. Fritsch-Carlson 单调性约束 Hermite 插值

---

*最后更新：2026-04-18*
