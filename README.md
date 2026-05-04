<p align="center">
  <img src="" width="120" alt="Welding Robot" />
</p>

<h1 align="center">Welding Robot</h1>
<h3 align="center">6 轴工业机器人焊接离线仿真系统</h3>

<p align="center">
  <img src="https://img.shields.io/badge/Unity-2022_LTS-222222?logo=unity" alt="Unity" />
  <img src="https://img.shields.io/badge/C%23-.NET_Standard_2.1-239120?logo=csharp" alt="C#" />
  <img src="https://img.shields.io/badge/Robot-FANUC_M10_iD-FF6600" alt="FANUC" />
  <img src="https://img.shields.io/badge/Status-毕设完成-green" alt="Status" />
</p>

---

## 这是什么？

一只 6 轴工业机器人在虚拟空间里做焊接任务的全流程仿真。

以 **FANUC M10-iD** 为建模目标，基于 MDH 运动学参数，从读入焊缝定义到关节空间轨迹执行，中间自动完成路径规划、碰撞检测、重规划避障，最后导出带有熔池可视化的完整仿真结果。

不是接真机的那种 —— 这是纯**离线仿真**，目的是在数字空间里把焊接过程先跑通、跑好看、跑安全。

---

## 一眼看懂架构

```
┌─────────────────────────────────────────────┐
│                  App Layer                   │
│         AppManager · KeyInput · UI           │
└────────────────────┬────────────────────────┘
                     │
┌────────────────────▼────────────────────────┐
│             Simulation Core                  │
│  SimulationContext · Clock(10ms) · StateMachine│
│  Idle ←→ Work ←→ Succeed/Fail ←→ Replay      │
│              ↕              ↕                  │
│         Joint · TCP  (遥操作)                  │
└────────────────────┬────────────────────────┘
                     │
     ┌───────────────┼───────────────┐
     ▼               ▼               ▼
┌─────────┐   ┌──────────┐   ┌──────────┐
│Kinematics│   │  Welding  │   │Collision │
│ FK · IK │   │ Path · Task│   │ Monitor  │
│ MDH模型 │   │ Trajectory│   │ 3组检测  │
└─────────┘   └──────────┘   └──────────┘
     │               │               │
     └───────────────┼───────────────┘
                     ▼
┌─────────────────────────────────────────────┐
│            Visualization Layer               │
│  WeldSeam · TcpPath · Collider · MoltenPool  │
└─────────────────────────────────────────────┘
```

四个层级各司其职：**App** 管人机交互，**Simulation** 做调度核心，**Kinematics / Welding / Collision** 是三大计算引擎，**Visualization** 把计算结果画在 Unity 场景里。

---

## 功能清单

| 子系统 | 干什么的 | 亮点 |
|--------|---------|------|
| **运动学** | FK (MDH) + IK (3种算法) | 解析法多解 / CCD迭代 / Jacobian Transpose，一键切换 |
| **路径规划** | TCP 空间路径生成 | 三种接近策略 (Direct/Safe/RRT) + 包角弧过渡 + PathSmoother |
| **轨迹规划** | 关节空间时间序列 | Cubic Hermite 样条 + Fritsch-Carlson 单调约束 + 速度校验 |
| **碰撞检测** | 实时距离查询 | Body / Gun / Self 三组独立，Box×Capsule 混合检测 |
| **重规划** | 碰撞预警自动避障 | RRT + Shadow Robot 校验 + 失败降级 Safe |
| **熔池可视化** | 焊道条带渲染 | 射线检测工件表面 + 三角条带 Mesh + 宽度反比线速度 |
| **仿真回放** | 复现历史仿真 | 逐帧回放 + 熔池同步 |
| **数据导出** | JSON 结果输出 | 含关节角/角速度/角加速度/重规划记录 |
| **遥操作** | 手动控制 | 单关节旋转 / TCP 6DOF |

---

## 运动学

### FK — 正向运动学

纯 MDH（Modified Denavit-Hartenberg），变换顺序固定为：

```
T_i = RotZ(θ_i) · TransZ(d_i) · TransX(a_i) · RotX(α_i)
```

串联 6 个关节得到末端法兰位姿，再叠加 TCP 偏移和 Tool Rotation 得到焊枪尖端的世界位姿。

### IK — 逆向运动学

三种算法对应三种使用场景：

| 算法 | 适合场景 | 为什么 |
|------|---------|--------|
| `ANALYTIC` (默认) | 批量规划 | 闭式解，速度快，产生 8 组候选解 |
| `CCD` | 连续路径 | 从末关节往回迭代，收敛平滑 |
| `JT` (Jacobian Transpose) | 精细控制 | 数值稳定，含姿态误差项 |

解析法处理了**肘上/肘下**和**腕部翻转**两种多解情况，通过 `IKResult.GetBestSolution()` 自动选取离当前姿态最近的一组解。

---

## 路径规划 — 三位一体

`TcpPathPlanner` 负责调度，按顺序委托给三个子规划器：

```
当前位置 ──[ApproachPathPlanner]──→ 焊缝起点
焊缝起点 ──[WeldPathPlanner]──────→ 焊缝终点（含包角弧）
焊缝终点 ──[ApproachPathPlanner]──→ 安全位置
```

### 接近路径策略

`ApproachPathPlanner` 提供三种策略，按场景选用：

| 策略 | 原理 | 适用场景 |
|------|------|---------|
| **Direct** | TCP 空间直线插值 | 无障碍时最快 |
| **Safe** | 先抬升到安全高度 → 水平移动 → 下降 | 简单障碍，稳健兜底 |
| **RRT** | 随机采样 + 树生长 + 路径提取 | 复杂障碍环境 |

RRT 的结果会经过 **PathSmoother** 后处理，四种力联合优化：

```
引力(缩短) → 拉直力(去锯齿) → 平滑力(曲率连续) → 排斥力(远离障碍)
       ↓ 每步
  IK求解 + 碰撞检验 ✅
```

### 焊接路径

直线焊缝 (`LineSeam`) 和弧线焊缝 (`ArcSeam`) 做均匀采样。**包角圆弧**是亮点——`WeldTask.Optimize()` 会自动识别相邻焊缝夹角，计算切点和圆弧中心，插入平滑过渡段。

---

## 轨迹规划 — 从空间到时间

TCP 路径点 → 时间轴上的关节角序列。

### 插值算法

| 方式 | 特点 |
|------|------|
| 线性插值 | 匀速转动，简单 |
| **Cubic Hermite 样条** (默认) | Fritsch-Carlson 单调约束，不超调不震荡 |

### 时间分配

每段轨迹用时取两种约束的较大值：

```
duration = max(
    TCP路径长度 / 焊接速度,                ← 线速度约束
    最大关节转角 / 关节最大角速度 × 1.2    ← 关节速度约束
)
```

### 速度校验

规划结果逐段检查，超出关节物理极限则标记 `JointSpeedLimitViolated`，进入 Fail 状态。

---

## 碰撞检测 — 三组独立

```
         ┌──── Body ────┐  J0~J6 连杆 vs 环境（工作台+工件）
Monitor ─┼──── Gun ─────┤  焊枪 vs 环境（阈值更小，焊枪贴近工件）
         └──── Self ────┘  前臂+腕部+焊枪 vs 底座+大臂+小臂（自体）
```

### 距离等级

| 等级 | 含义 | 触发动作 |
|------|------|---------|
| `Safe` | 距离充裕 | — |
| `Warning` | 进入预警区 | 触发 RRT 重规划 |
| `Collision` | 穿透或过近 | Work→Fail / 遥操作→回滚 |

核心算法：`Physics.ComputePenetration` 判断穿透，双向 `ClosestPoint` 计算分离距离。Box 和 Capsule 碰撞体通过 `ColliderShape` 接口统一处理。

配色约定：🟡 机器人 / 🩵 工作台 / 🟢 工件

---

## 熔池可视化

`MoltenPoolVisualizer` 每帧的工作流程：

```
1. TCP 左右偏移 r，向焊接方向发射两根射线
2. 射线命中工件表面 → 两个交点
3. 上帧 + 本帧共 4 顶点 → 构建 2 个三角形
4. 条带宽度与线速度成反比（快→窄 / 慢→宽）
```

每段焊接 = 一个 `MoltenPoolStrip`（独立 Mesh + GameObject），焊接开始创建，结束冻结。

---

## 仿真状态机

```
                    ┌──────────┐
        ┌──────────→│   Idle   │←──────────┐
        │           └────┬─────┘           │
        │                │ Space            │
        │           ┌────▼─────┐           │
   ┌────┴─────┐     │   Work   │     ┌────┴─────┐
   │  Joint   │     └────┬─────┘     │   TCP    │
   │ (遥操作) │          │           │ (遥操作) │
   └──────────┘     ┌────┴─────┐     └──────────┘
                    ├──────────┤
               ┌────▼──┐  ┌───▼────┐
               │Succeed│  │  Fail  │
               └───┬───┘  └───┬────┘
                   │   Tab     │
                   └──→┌───────▼──┐
                       │  Replay  │
                       └──────────┘
```

7 个状态，核心流程 Idle→Work→Succeed/Fail，Joint/TCP 手动控制，Replay 从终态进入。

---

## 坐标系

写代码时必须记住的映射：

| | Data 坐标系 | Unity 坐标系 |
|--|------------|-------------|
| **Forward** | +X | +Z |
| **Left** | +Y | -X |
| **Up** | +Z | +Y |
| **用途** | JSON 输入、内部计算 | 场景渲染 |

转换函数：`MathUtil.D2UPosition / D2URotation`。旋转通过基向量映射实现，比欧拉角直接映射可靠。

---

## 任务配置 (JSON)

放在 `Welding Tasks/` 目录下的 JSON 文件：

```json
{
  "TaskName": "测试任务",
  "UserOrigin": { "x": 1.0, "y": 0.0, "z": 0.3 },
  "Workpiece": {
    "FileName": "workpiece.glb",
    "Position": { "x": 0, "y": 0, "z": 0.02 },
    "Rotation": { "x": 0, "y": 0, "z": 0 },
    "Scale": { "x": 0.5, "y": 0.5, "z": 0.5 },
    "Colliders": [
      { "Min": { "x": -0.3, "y": -0.3, "z": -0.02 },
        "Max": { "x": 0.3, "y": 0.3, "z": 0.0 } }
    ]
  },
  "WeldSeams": [
    {
      "Type": "Line",
      "ID": 1, "Name": "Seam_1",
      "Speed": 0.04, "GunAngle": 45, "GunDistance": 0.02,
      "StartPoint": { "x": -0.16, "y": -0.15, "z": 0.02 },
      "EndPoint": { "x": -0.14, "y": -0.15, "z": 0.02 },
      "Normal": { "x": 0, "y": 0, "z": 1 },
      "CornerRadius": 0.005
    }
  ]
}
```

| 字段 | 说明 |
|------|------|
| `Type` | `"Line"` 或 `"Arc"` |
| `GunAngle` | 焊枪与法向夹角 (度) |
| `GunDistance` | 焊枪到焊缝距离 (m) |
| `CornerRadius` | 包角半径，>0 则自动圆弧过渡 |
| `Normal` | 焊缝面法向 (Data坐标系) |

---

## 导出数据

仿真完成后导出 `WeldResultData` 含：

```
Frames (10ms采样):
  { timestamp, joints[6], tcpPose, tcpSpeed,
    jointVelocities[6], jointAccelerations[6] }

Points[]:          规划路径点
ReplanRecords[]:   重规划记录 (时机/耗时/位姿/是否成功)
```

---

## 键盘控制

| 按键 | 功能 |
|------|------|
| `Space` | 开始 / 暂停 |
| `Tab` | 回放 |
| `R` | 重置 |
| `V` | 切换摄像头（正视/俯视） |
| `Shift` | 切换 IK 算法 |

---

## 技术栈

| 项目 | 说明 |
|------|------|
| Unity 2022 LTS | 引擎 |
| C# (.NET Standard 2.1) | 语言 |
| glTFast | GLB 加载 |
| Newtonsoft.Json | 序列化 |
| TextMesh Pro | UI 文字 |

---

## 项目结构

```
Welding Robot/
├── Assets/
│   ├── Scripts/
│   │   ├── App/              # 应用入口
│   │   ├── Simulation/       # 仿真核心
│   │   ├── Robot/            # 运动学 + 模型
│   │   ├── Welding/          # 任务/路径/轨迹
│   │   ├── Collision/        # 碰撞检测
│   │   ├── Visualization/    # 场景可视化
│   │   └── Utils/            # 数学工具
│   ├── Prefabs/
│   ├── Scenes/
│   └── Config/
├── Welding Tasks/             # JSON 任务
├── Packages/
└── ProjectSettings/
```

---

## 代码导航

| 想知道... | 读这个 |
|-----------|--------|
| 系统怎么跑起来 | `Simulation/SimulationContext.cs` |
| 状态切换逻辑 | `Simulation/SimulationStateMachine.cs` |
| FK (MDH 正解) | `Robot/Kinematics/FK.cs` |
| IK (解析/CCD/JT) | `Robot/Kinematics/IK.cs` |
| 路径调度中心 | `Welding/Path/TcpPathPlanner.cs` |
| RRT 避障 | `Welding/Path/RrtPathPlanner.cs` |
| 路径平滑 | `Welding/Path/PathSmoother.cs` |
| APF 势场 | `Welding/Path/ApfPathPlanner.cs` |
| 轨迹规划 | `Welding/Trajectory/TrajectoryPlanner.cs` |
| 碰撞监控 | `Collision/CollisionMonitor.cs` |
| 熔池渲染 | `Visualization/MoltenPoolVisualizer.cs` |
| 坐标转换 | `Utils/MathUtil.cs` |

---

<p align="center">
  <em>Built with 🐉 by Cherry · 2026 届本科毕业设计</em>
</p>
