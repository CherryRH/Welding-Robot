# Welding Robot — 焊接机器人仿真系统

> Unity 2022 LTS | 基于 MDH 参数模型的 6 轴工业机器人仿真

---

## 项目概述

本项目是一个 **Unity 焊接机器人离线仿真系统**，模拟 FANUC M10-iD 等 6 轴工业机器人在工件上执行焊接任务的全流程。系统以 Data 右手坐标系（forward=+X, left=+Y, up=+Z）为数据核心，通过 Unity 场景实现可视化。

---

## 架构总览

```
App
├── Managers          # 应用管理器
│   ├── AppManager           # 多仿真上下文切换
│   └── KeyInputManager      # 键盘输入管理
│
└── UI               # 界面
    ├── AppView              # 应用视图
    ├── ControlView         # 控制面板视图
    └── SimulationView      # 仿真视图

Simulation
├── SimulationContext        # 仿真上下文（核心调度器）
├── SimulationClock          # 仿真时钟（固定步长 10ms）
├── SimulationStateMachine   # 状态机
├── SimulationStateBase      # 状态基类 + 各状态实现
│                            (Idle / Work / Succeed / Fail / Joint / TCP)

Robot
├── Model                   # 机器人模型
│   ├── RobotModel          # 机器人模型（关节状态、TCP、运动学）
│   └── JointModel          # 单关节模型
│
├── Kinematics              # 运动学
│   ├── FK                  # 正向运动学（MDH 变换）
│   ├── IK                  # 逆向运动学（CCD / Jacobian Transpose / 解析法）
│   ├── IKResult            # IK 结果（多解支持）
│   └── JointParameters     # 关节 MDH 参数

Welding
├── Data                    # 数据层
│   ├── WeldTaskData        # 焊接任务数据
│   ├── WeldSeamData        # 焊缝数据
│   ├── WorkpieceData       # 工件数据
│   └── WeldTaskDataLoader  # JSON 文件加载器
│
├── Object                  # 焊接对象
│   ├── WeldSeam            # 焊缝基类
│   ├── LineSeam            # 直线焊缝
│   ├── ArcSeam             # 弧焊焊缝
│   └── Workpiece           # 工件
│
├── Path                    # TCP 路径规划
│   ├── TcpPathPlanner      # TCP 路径调度器
│   ├── TcpPathPoint        # TCP 路径点
│   ├── ApproachPathPlanner  # 接近路径规划
│   ├── WeldPathPlanner     # 焊接路径规划
│   └── AdjustPathPlanner   # 调整路径规划（重规划时使用）
│
├── Task                    # 任务层
│   ├── WeldTask            # 焊接任务
│   └── WeldTaskPlanState   # 任务规划状态
│
└── Trajectory              # 关节空间轨迹
    ├── Trajectory           # 轨迹（List 存储，用 index 维护当前段）
    ├── TrajectorySegment    # 轨迹段
    ├── TrajectoryPlanner    # 轨迹规划器（速度限制检查）
    └── IJointInterpolator   # 关节插值接口
        ├── LinearJointInterpolator          # 线性插值
        └── CubicHermiteSegmentInterpolator  # 三次 Hermite 样条插值

DataFlow                     # 数据绑定层（Unity ↔ 数据模型）
├── RobotBinder              # 机械臂绑定（关节角度 ↔ Transform）
├── WorkbenchBinder          # 工作台/工件绑定
└── EffectBinder            # 焊接特效绑定

Visualization                # 可视化层
├── ColliderVisualizer       # 碰撞箱可视化（GL 渲染，Play 模式）
├── TcpPathVisualizer        # TCP 路径点可视化
└── WeldSeamVisualizer       # 焊缝可视化（Debug.DrawLine）

Utils                        # 工具类
├── MathUtil                 # 数学工具（坐标转换、MDH、欧拉角）
├── JsonUtil                 # JSON 序列化
├── LogUtil                  # 日志工具
├── FileUtil                 # 文件工具
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
        ┌──────┐  Space   ┌──────┐
   ┌──►│ Idle │─────────►│ Work │
   │   └──┬───┘          └──┬───┘
   │      │     Esc         │     轨迹执行完毕
   │      │◄───────────────┘
   │      │  Esc/Space
   │      ▼
   │   ┌──────┐  Space   ┌─────────┐
   │◄─┤ Fail │◄─────────│ Succeed │
   │   └──────┘          └─────────┘
   │
   │ Shift  ┌────────┐
   └────────┤ Joint  │  Esc  ──► Idle（关节遥操作）
   │        └───┬────┘
   │            │ Shift
   │            ▼
   │        ┌───────┐
   └────────┤  TCP  │  Esc  ──► Idle（TCP 遥操作）
            └───────┘
```

**状态说明：**
- `Idle` — 空闲，可按 Space 开始
- `Work` — 执行焊接任务
- `Succeed` — 任务成功
- `Fail` — 任务失败（重规划超过次数限制）
- `Joint` — 关节遥操作模式
- `TCP` — TCP 遥操作模式

---

## 运动学

### 正向运动学（FK）
- 基于 **MDH 参数**（Modified Denavit-Hartenberg）
- 变换顺序：α → a → θ → d
- 支持 Tool Rotation 偏置

### 逆向运动学（IK）
三种算法可切换（按 Shift）：

| 算法 | 类型 | 特点 |
|------|------|------|
| `ANALYTIC` | 解析法 | 速度快，有多解（肘上/肘下、翻转腕） |
| `CCD` | 循环坐标下降 | 迭代收敛，适合连续路径 |
| `JT` | Jacobian 转置 | 数值稳定，适合精细控制 |

---

## 轨迹规划

### 规划流程
```
焊接任务 → TCP路径规划 → 轨迹规划 → 关节运动
(焊缝)    (TcpPathPlanner) (TrajectoryPlanner) (FK)
```

### 数据结构
- **TcpPathPlanner**：`LinkedList<TcpPathPoint>` 管理 TCP 路径点
- **Trajectory**：`LinkedList<TrajectorySegment>` 管理轨迹段，动态出队执行

### 轨迹插值
- **线性插值**：恒定关节速度
- **三次 Hermite 样条**：平滑起止速度，支持单调速度限制（Fritsch–Carlson）

### 关键特性
- **速度限制检查**：规划时验证关节角速度是否超限
- **重规划机制**：遇到奇异点/速度超限 → 插入调整路径（Adjust）
- **奇异点处理**：J5=0° 时进入奇异姿态，自动归零后离开
- **腕部翻转**：允许腕部 180° 翻转以获得更优姿态

---

## 碰撞系统（规划中）

### 总体方案
直接基于 **Unity 物理系统**（Collider / Collision）实现，不另起炉灶。

### 架构
```
CollisionMonitor            # 碰撞监测器（挂载到 SimulationContext）
├── CheckSelfCollision()    # 自体碰撞检测
├── CheckEnvironmentCollision() # 环境碰撞检测
└── Unity Events           # 碰撞事件回调
```

### 碰撞对约定
```
自体碰撞（只检测关键对）：
  J5 / J6 / 焊枪  vs  J1(底座) / J2 / J3

环境碰撞：
  机械臂各部分  vs  工作台 + 工件
```

### 颜色约定
| 碰撞箱类型 | 颜色 |
|-----------|------|
| 机械臂 | 黄色 |
| 工作台 | 青色 |
| 工件 | 绿色 |

---

## 关键文件索引

| 功能 | 文件 |
|------|------|
| 应用入口 | `AppManager.cs` |
| 仿真核心 | `SimulationContext.cs` |
| 状态机 | `SimulationStateMachine.cs` + `SimulationStateBase.cs` |
| 仿真时钟 | `SimulationClock.cs` |
| 正向运动学 | `FK.cs` |
| 逆向运动学 | `IK.cs` |
| 轨迹执行 | `Trajectory.cs` |
| 轨迹规划 | `TrajectoryPlanner.cs` |
| 关节插值 | `IJointInterpolator.cs` |
| TCP 路径 | `TcpPathPlanner.cs` |
| 碰撞可视化 | `ColliderVisualizer.cs` |
| 坐标转换 | `MathUtil.cs` |
| 工件加载 | `WorkbenchBinder.cs` |
| 机械臂绑定 | `RobotBinder.cs` |

---

## 正在进行 / 待完成

- [ ] **CollisionMonitor** — 基于 Unity 物理系统实现的碰撞监测模块
- [ ] **WeldResultDataWriter** — 焊接结果数据导出
- [ ] 焊接过程数据记录（动态整合焊接数据，为输出做准备）

---

## 配置说明

### RobotConfig（ScriptableObject）
- MDH 参数（Alpha, A, Theta, D）
- 关节角度限制（Min, Max）及最大角速度
- TCP 偏置、法兰偏移、Tool Rotation
- 遥操作速度参数

### 焊接任务文件（JSON）
路径：`Welding Tasks/*.json`
```json
{
  "TaskName": "任务名",
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
      "StartPoint": { "x": -0.2, "y": 0, "z": 0 },
      "EndPoint": { "x": -0.1, "y": 0, "z": 0 }
    }
  ]
}
```
