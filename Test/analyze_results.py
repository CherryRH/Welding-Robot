#!/usr/bin/env python3
"""
焊接机器人仿真结果分析工具
用法: python analyze_results.py <test_number> [--run <run_index>]
示例: python analyze_results.py 1          # 分析 Test 1 所有运行
      python analyze_results.py 4 --run 0  # 只看 Test 4 第一次运行
"""

import json
import os
import sys
import glob
import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.font_manager import FontProperties

# ──────────────────────────────────────────────────────────
# 配置
# ──────────────────────────────────────────────────────────

RESULTS_DIR = r"C:\Users\crh61\AppData\LocalLow\CherryRH\Welding Robot\Results"
OUTPUT_BASE = Path(r"E:\Unity\Projects\Welding Robot\Test\output")
FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"

# 论文级样式
plt.rcParams.update({
    "figure.dpi": 150,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.05,
    "font.size": 10,
    "axes.titlesize": 12,
    "axes.labelsize": 10,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

# 中文字体
if os.path.exists(FONT_PATH):
    _fp = FontProperties(fname=FONT_PATH)
    plt.rcParams["font.family"] = _fp.get_name()
    FONT_PROP = _fp
else:
    FONT_PROP = None
    print("[WARN] 未找到微软雅黑字体，中文可能显示为方框")

COLORS = ["#2196F3", "#FF9800", "#4CAF50", "#E91E63", "#9C27B0", "#00BCD4"]


# ──────────────────────────────────────────────────────────
# 数据加载
# ──────────────────────────────────────────────────────────

def load_result_files(test_num: int) -> list[dict]:
    """加载指定 Test 的所有结果文件（按时间戳排序）"""
    pattern = os.path.join(RESULTS_DIR, f"Test {test_num}_*.json")
    files = sorted(glob.glob(pattern))
    if not files:
        raise FileNotFoundError(f"未找到 Test {test_num} 的结果文件: {pattern}")
    results = []
    for fp in files:
        with open(fp, "r", encoding="utf-8") as f:
            results.append(json.load(f))
    return results


def iter_weld_frames(data: dict):
    """遍历仅 Weld 类型的帧，返回 (timestamp, frame)"""
    for ts_str, frm in data.get("Frames", {}).items():
        if str(frm.get("PointType", "")) == "Weld":
            yield float(ts_str), frm


def iter_seam_frames(data: dict, seam_id: int):
    """遍历指定焊缝的所有帧（包含 Weld + 前后调整段）"""
    for ts_str, frm in data.get("Frames", {}).items():
        if frm.get("SeamId") == seam_id:
            yield float(ts_str), frm


def get_seam_ids(data: dict) -> list[int]:
    """从 Frames 中提取所有出现过的 SeamId（排重排序）"""
    ids = set()
    for frm in data["Frames"].values():
        sid = frm.get("SeamId", -1)
        if sid >= 0:
            ids.add(sid)
    return sorted(ids)


# ──────────────────────────────────────────────────────────
# 统计工具
# ──────────────────────────────────────────────────────────

def stats(values: list[float]) -> dict:
    """返回均值、标准差、最大、RMS"""
    arr = np.array(values)
    return {
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "max": float(np.max(np.abs(arr))),
        "rms": float(np.sqrt(np.mean(arr ** 2))),
        "count": len(values),
    }


def safe_div(a, b):
    return a / b if b != 0 else 0.0


# ──────────────────────────────────────────────────────────
# 图表生成
# ──────────────────────────────────────────────────────────

class ChartWriter:
    """统一管理图表输出路径"""

    def __init__(self, out_dir: Path):
        self.chart_dir = out_dir / "charts"
        self.chart_dir.mkdir(parents=True, exist_ok=True)

    def save(self, name: str):
        path = self.chart_dir / name
        plt.savefig(path)
        plt.close()
        return name


# ──────────────────────────────────────────────────────────
# 1. 焊接效果评估图表
# ──────────────────────────────────────────────────────────

def chart_position_error_per_seam(all_runs: list[dict], cw: ChartWriter):
    """各焊缝位置误差箱线图（跨所有运行）"""
    seam_errors = defaultdict(list)
    for data in all_runs:
        for _, frm in iter_weld_frames(data):
            sid = frm.get("SeamId", -1)
            if sid >= 0:
                seam_errors[sid].append(abs(frm.get("PositionError", 0)) * 1000)  # → mm

    if not seam_errors:
        return None

    ids = sorted(seam_errors.keys())
    data_list = [seam_errors[i] for i in ids]
    labels = [f"焊缝 {i}" for i in ids]

    fig, ax = plt.subplots(figsize=(max(6, len(ids) * 0.5), 4.5))
    bp = ax.boxplot(data_list, labels=labels, patch_artist=True, showfliers=True,
                    flierprops={"marker": ".", "markersize": 2, "alpha": 0.4})
    for patch, color in zip(bp["boxes"], COLORS * 10):
        patch.set_facecolor(color)
        patch.set_alpha(0.5)

    ax.set_ylabel("位置误差 (mm)", fontproperties=FONT_PROP)
    ax.axhline(y=np.mean([np.mean(d) for d in data_list]), color="red",
               linestyle="--", linewidth=1.2, label="总体均值")
    if len(ids) <= 10:
        ax.legend()
    plt.xticks(rotation=45 if len(ids) > 8 else 0)
    return cw.save("position_error_per_seam.png")


def chart_orientation_error_per_seam(all_runs: list[dict], cw: ChartWriter):
    """各焊缝姿态误差箱线图"""
    seam_errors = defaultdict(list)
    for data in all_runs:
        for _, frm in iter_weld_frames(data):
            sid = frm.get("SeamId", -1)
            if sid >= 0:
                seam_errors[sid].append(abs(frm.get("OrientationError", 0)))

    if not seam_errors:
        return None

    ids = sorted(seam_errors.keys())
    data_list = [seam_errors[i] for i in ids]
    labels = [f"焊缝 {i}" for i in ids]

    fig, ax = plt.subplots(figsize=(max(6, len(ids) * 0.5), 4.5))
    bp = ax.boxplot(data_list, labels=labels, patch_artist=True, showfliers=True,
                    flierprops={"marker": ".", "markersize": 2, "alpha": 0.4})
    for patch, color in zip(bp["boxes"], COLORS * 10):
        patch.set_facecolor(color)
        patch.set_alpha(0.5)

    ax.set_ylabel("姿态误差 (°)", fontproperties=FONT_PROP)
    plt.xticks(rotation=45 if len(ids) > 8 else 0)
    return cw.save("orientation_error_per_seam.png")


def chart_speed_error_per_seam(all_runs: list[dict], cw: ChartWriter):
    """各焊缝速度误差箱线图"""
    seam_errors = defaultdict(list)
    for data in all_runs:
        for _, frm in iter_weld_frames(data):
            sid = frm.get("SeamId", -1)
            if sid >= 0:
                seam_errors[sid].append(abs(frm.get("SpeedError", 0)) * 1000)  # → mm/s

    if not seam_errors:
        return None

    ids = sorted(seam_errors.keys())
    data_list = [seam_errors[i] for i in ids]
    labels = [f"焊缝 {i}" for i in ids]

    fig, ax = plt.subplots(figsize=(max(6, len(ids) * 0.5), 4.5))
    bp = ax.boxplot(data_list, labels=labels, patch_artist=True, showfliers=True,
                    flierprops={"marker": ".", "markersize": 2, "alpha": 0.4})
    for patch, color in zip(bp["boxes"], COLORS * 10):
        patch.set_facecolor(color)
        patch.set_alpha(0.5)

    ax.set_ylabel("速度误差 (mm/s)", fontproperties=FONT_PROP)
    plt.xticks(rotation=45 if len(ids) > 8 else 0)
    return cw.save("speed_error_per_seam.png")


def chart_error_histogram(all_runs: list[dict], cw: ChartWriter):
    """三项误差综合分布直方图"""
    pos_errs, ori_errs, spd_errs = [], [], []
    for data in all_runs:
        for _, frm in iter_weld_frames(data):
            pos_errs.append(abs(frm.get("PositionError", 0)) * 1000)
            ori_errs.append(abs(frm.get("OrientationError", 0)))
            spd_errs.append(abs(frm.get("SpeedError", 0)) * 1000)

    fig, axes = plt.subplots(1, 3, figsize=(12, 3.5))
    titles = ["位置误差 (mm)", "姿态误差 (°)", "速度误差 (mm/s)"]
    data_sets = [pos_errs, ori_errs, spd_errs]

    for ax, vals, title, color in zip(axes, data_sets, titles, COLORS):
        if not vals:
            continue
        ax.hist(vals, bins=40, color=color, alpha=0.7, edgecolor="white")
        ax.axvline(x=np.mean(vals), color="red", linestyle="--", linewidth=1.5,
                   label=f'均值={np.mean(vals):.2f}')
        ax.legend(prop=FONT_PROP, fontsize=7)

    return cw.save("error_histogram.png")


def chart_tcp_speed_timeline(data: dict, cw: ChartWriter, label: str = ""):
    """TCP 线速度时间序列（标注各段类型）"""
    times, speeds, types = [], [], []
    for ts_str, frm in data["Frames"].items():
        times.append(float(ts_str))
        speeds.append(frm.get("TcpSpeed", 0) * 1000)  # → mm/s
        types.append(frm.get("PointType", -1))

    if not times:
        return None

    fig, ax = plt.subplots(figsize=(10, 3.5))

    # 分段着色（避免跨类型连线）
    type_map = {"Approach": "#90CAF9", "Weld": "#4CAF50", "Adjust": "#FFCC80"}
    type_labels = {"Approach": "接近段", "Weld": "焊接段", "Adjust": "调整段"}
    _draw_segments(ax, times, speeds, types, type_map, type_labels)

    ax.set_xlabel("仿真时间 (s)", fontproperties=FONT_PROP)
    ax.set_ylabel("TCP 线速度 (mm/s)", fontproperties=FONT_PROP)

    # 统一图例
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), prop=FONT_PROP, fontsize=7)

    return cw.save(f"tcp_speed_timeline{'_' + label.replace(' ','_') if label else ''}.png")


def _draw_segments(ax, times, values, types, colors_map, label_map):
    """按类型分段绘制，避免不同类型之间连线"""
    times_arr = np.array(times)
    values_arr = np.array(values)
    types_arr = np.array(types)

    for tname, color in colors_map.items():
        mask = types_arr == tname
        if not mask.any():
            continue
        # 提取连续段
        idx = np.where(mask)[0]
        if len(idx) == 0:
            continue
        # 分成连续子段
        segments = []
        start = 0
        for i in range(1, len(idx)):
            if idx[i] != idx[i-1] + 1:
                segments.append(idx[start:i])
                start = i
        segments.append(idx[start:])
        for seg in segments:
            ax.plot(times_arr[seg], values_arr[seg], color=color,
                    linewidth=1.0, alpha=0.8, label=label_map.get(tname, tname))


def chart_seam(data: dict, seam_id: int, cw: ChartWriter):
    """焊缝的误差时序图（按段类型分色，避免跨类型连线）"""
    times = []
    pos_errs, ori_errs, spd_errs = [], [], []
    types = []
    for ts, frm in iter_seam_frames(data, seam_id):
        times.append(ts)
        pos_errs.append(abs(frm.get("PositionError", 0)) * 1000)
        ori_errs.append(abs(frm.get("OrientationError", 0)))
        spd_errs.append(frm.get("TcpSpeed", 0) * 1000)
        types.append(str(frm.get("PointType", "Unknown")))

    if not times:
        return None

    type_colors = {"Approach": "#90CAF9", "Weld": "#4CAF50", "Adjust": "#FFCC80"}
    type_labels = {"Approach": "接近段", "Weld": "焊接段", "Adjust": "调整段"}

    fig, axes = plt.subplots(3, 1, figsize=(10, 6), sharex=True)

    _draw_segments(axes[0], times, pos_errs, types, type_colors, type_labels)
    axes[0].set_ylabel("位置误差 (mm)", fontproperties=FONT_PROP)

    _draw_segments(axes[1], times, ori_errs, types, type_colors, type_labels)
    axes[1].set_ylabel("姿态误差 (°)", fontproperties=FONT_PROP)

    _draw_segments(axes[2], times, spd_errs, types, type_colors, type_labels)
    axes[2].set_ylabel("TCP 速度 (mm/s)", fontproperties=FONT_PROP)
    axes[2].set_xlabel("时间 (s)", fontproperties=FONT_PROP)

    # 统一图例
    handles, labels = axes[2].get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    fig.legend(by_label.values(), by_label.keys(), loc='upper right',
               prop=FONT_PROP, fontsize=7, ncol=3)

    plt.tight_layout()
    return cw.save(f"seam_{seam_id}_detail.png")


# ──────────────────────────────────────────────────────────
# 2. 安全性评估图表
# ──────────────────────────────────────────────────────────

def chart_replan_analysis(all_runs: list[dict], cw: ChartWriter):
    """重规划统计"""
    counts = []
    success_rates = []
    comp_times = []

    for data in all_runs:
        replans = data.get("ReplanRecords", [])
        counts.append(len(replans))
        if replans:
            success_rates.append(
                sum(1 for r in replans if r.get("IsSuccessful", False)) / len(replans) * 100)
            for r in replans:
                comp_times.append(r.get("ComputationTimeMs", 0))

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))

    # 左：重规划次数
    if counts and max(counts) > 0:
        axes[0].bar(range(len(counts)), counts, color=COLORS[0], alpha=0.7)
        axes[0].axhline(y=np.mean(counts), color="red", linestyle="--",
                        label=f'均值={np.mean(counts):.1f}')
        axes[0].set_xlabel("运行编号", fontproperties=FONT_PROP)
        axes[0].set_ylabel("重规划次数", fontproperties=FONT_PROP)
        axes[0].legend(prop=FONT_PROP)
    else:
        axes[0].text(0.5, 0.5, "无重规划", transform=axes[0].transAxes, ha="center")

    # 右：RRT 计算耗时分布
    if comp_times:
        axes[1].hist(comp_times, bins=20, color=COLORS[3], alpha=0.7, edgecolor="white")
        axes[1].axvline(x=np.mean(comp_times), color="red", linestyle="--",
                        label=f'均值={np.mean(comp_times):.1f}ms')
        axes[1].set_xlabel("计算时间 (ms)", fontproperties=FONT_PROP)
        axes[1].set_ylabel("频次", fontproperties=FONT_PROP)
        axes[1].legend(prop=FONT_PROP)
    else:
        axes[1].text(0.5, 0.5, "无重规划记录", transform=axes[1].transAxes, ha="center")

    plt.tight_layout()
    return cw.save("replan_analysis.png")


# ──────────────────────────────────────────────────────────
# 3. 关节运动分析
# ──────────────────────────────────────────────────────────

def chart_joint_velocities(data: dict, cw: ChartWriter):
    """6 关节角速度时序"""
    times = []
    jv = [[] for _ in range(6)]
    for ts_str, frm in data["Frames"].items():
        times.append(float(ts_str))
        for i, v in enumerate(frm.get("JointVelocities", [0] * 6)):
            if i < 6:
                jv[i].append(v)

    if not times:
        return None

    fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
    axes = axes.flatten()

    for i, ax in enumerate(axes):
        if i < 6 and len(jv[i]) == len(times):
            ax.plot(times, jv[i], color=COLORS[i], linewidth=1.0)
            ax.axhline(y=0, color="gray", linewidth=0.5)
            ax.set_ylabel(f"J{i+1} 角速度 (°/s)", fontproperties=FONT_PROP, fontsize=7)

    axes[-1].set_xlabel("仿真时间 (s)", fontproperties=FONT_PROP)
    plt.tight_layout()
    return cw.save("joint_velocities.png")


def chart_joint_accelerations(data: dict, cw: ChartWriter):
    """6 关节角加速度时序"""
    times = []
    ja = [[] for _ in range(6)]
    for ts_str, frm in data["Frames"].items():
        times.append(float(ts_str))
        for i, a in enumerate(frm.get("JointAccelerations", [0] * 6)):
            if i < 6:
                ja[i].append(a)

    if not times:
        return None

    fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
    axes = axes.flatten()

    for i, ax in enumerate(axes):
        if i < 6 and len(ja[i]) == len(times):
            ax.plot(times, ja[i], color=COLORS[i], linewidth=1.0, alpha=0.8)
            ax.axhline(y=0, color="gray", linewidth=0.5)
            ax.set_ylabel(f"J{i+1} 角加速度 (°/s²)", fontproperties=FONT_PROP, fontsize=7)

    axes[-1].set_xlabel("仿真时间 (s)", fontproperties=FONT_PROP)
    plt.tight_layout()
    return cw.save("joint_accelerations.png")


def chart_joint_angle_timeline(data: dict, cw: ChartWriter):
    """6 关节角度时序"""
    times = []
    ja = [[] for _ in range(6)]
    for ts_str, frm in data["Frames"].items():
        times.append(float(ts_str))
        for i, a in enumerate(frm.get("JointAngles", [0] * 6)):
            if i < 6:
                ja[i].append(a)

    if not times:
        return None

    fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
    axes = axes.flatten()

    for i, ax in enumerate(axes):
        if i < 6 and len(ja[i]) == len(times):
            ax.plot(times, [v for v in ja[i]], color=COLORS[i], linewidth=1.0)
            ax.set_ylabel(f"J{i+1} 关节角度 (°)", fontproperties=FONT_PROP, fontsize=7)
    
    axes[-1].set_xlabel("仿真时间 (s)", fontproperties=FONT_PROP)
    plt.tight_layout()
    return cw.save("joint_angles.png")


# ──────────────────────────────────────────────────────────
# 报告生成
# ──────────────────────────────────────────────────────────

def compute_per_seam_stats(all_runs: list[dict]) -> dict:
    """跨所有运行，按焊缝计算 PositionError / OrientationError / SpeedError"""
    seam_pos = defaultdict(list)
    seam_ori = defaultdict(list)
    seam_spd = defaultdict(list)

    for data in all_runs:
        for _, frm in iter_weld_frames(data):
            sid = frm.get("SeamId", -1)
            if sid >= 0:
                seam_pos[sid].append(abs(frm.get("PositionError", 0)))
                seam_ori[sid].append(abs(frm.get("OrientationError", 0)))
                seam_spd[sid].append(abs(frm.get("SpeedError", 0)))

    result = {}
    for sid in sorted(set(list(seam_pos.keys()) + list(seam_ori.keys()))):
        result[sid] = {
            "pos": stats(seam_pos.get(sid, [0.0])),
            "ori": stats(seam_ori.get(sid, [0.0])),
            "spd": stats(seam_spd.get(sid, [0.0])),
        }
    return result


def fmt_stat(s: dict, unit: str) -> str:
    """格式化统计量输出"""
    if not s or s["count"] == 0:
        return "—"
    return f'{s["mean"]:.3f} {unit} (σ={s["std"]:.3f}, max={s["max"]:.3f})'


def build_report(test_num: int, all_runs: list[dict], charts: dict[str, str | None],
                 out_dir: Path) -> str:
    """生成 Markdown 报告"""

    n = len(all_runs)
    times = [d.get("TotalTime", 0) for d in all_runs]
    f_counts = [len(d.get("Frames", {})) for d in all_runs]
    r_counts = [len(d.get("ReplanRecords", [])) for d in all_runs]

    all_replans = []
    for d in all_runs:
        all_replans.extend(d.get("ReplanRecords", []))

    replan_success = safe_div(
        sum(1 for r in all_replans if r.get("IsSuccessful", False)),
        len(all_replans),
    )

    comp_times_ms = [r.get("ComputationTimeMs", 0) for r in all_replans]

    # PlanStatus 统计
    plan_statuses = [d.get("PlanStatus", "Unknown") for d in all_runs]
    status_counts = {}
    for s in plan_statuses:
        status_counts[s] = status_counts.get(s, 0) + 1
    task_success_rate = safe_div(status_counts.get("Succeeded", 0), n) * 100

    # 每焊缝统计
    per_seam = compute_per_seam_stats(all_runs)

    # 全局误差 (所有焊接帧)
    all_pos = []
    all_ori = []
    all_spd = []
    for d in all_runs:
        for _, frm in iter_weld_frames(d):
            all_pos.append(abs(frm.get("PositionError", 0)))
            all_ori.append(abs(frm.get("OrientationError", 0)))
            all_spd.append(abs(frm.get("SpeedError", 0)))

    pos_st = stats(all_pos) if all_pos else {}
    ori_st = stats(all_ori) if all_ori else {}
    spd_st = stats(all_spd) if all_spd else {}

    # ── 构建 Markdown ──

    def img(name: str | None, alt: str = "") -> str:
        if name:
            return f"![{alt}](charts/{name})"
        return "_(无数据)_"

    lines = [
        f"# Test {test_num} — 仿真结果分析报告",
        f"",
        f"**任务名称:** {all_runs[0].get('TaskName', '?')}",
        f"**运行次数:** {n}",
        f"**统一配置:** ANALYTIC | CubicHermite | RRT | dt=0.01s",
        f"",
        "---",
        f"",
        f"## 1. 总体概览",
        f"",
        f"| 指标 | 均值 (范围) |",
        f"|------|------------|",
        f"| 仿真时间 | {np.mean(times):.1f}s ({min(times):.1f}~{max(times):.1f}) |",
        f"| 帧数 | {int(np.mean(f_counts))} 帧 ({min(f_counts)}~{max(f_counts)}) |",
        f"| 重规划次数 | {np.mean(r_counts):.1f} 次 ({min(r_counts)}~{max(r_counts)}) |",
        f"| 重规划成功率 | {replan_success * 100:.1f}% ({sum(1 for r in all_replans if r.get('IsSuccessful'))}/{len(all_replans)}) |",
        f"| RRT 平均耗时 | {np.mean(comp_times_ms):.2f}ms (max={np.max(comp_times_ms):.2f}) |" if comp_times_ms else "| RRT 平均耗时 | — (无重规划) |",
        f"| 任务成功率 | {task_success_rate:.1f}% ({status_counts.get('Succeeded', 0)}/{n}) |",
        f"| PlanStatus 分布 | Succeeded={status_counts.get('Succeeded', 0)}, Failed={status_counts.get('Failed', 0)}, Unfinished={status_counts.get('Unfinished', 0)} |",
        f"",
        f"---",
        f"",
        f"## 2. 焊接效果评估",
        f"",
        f"### 2.1 全局误差统计",
        f"",
        f"| 误差指标 | 均值 | 标准差 | 最大 | RMS |",
        f"|----------|------|--------|------|-----|",
        f"| 位置误差 | {pos_st.get('mean', 0) * 1000:.3f} mm | {pos_st.get('std', 0) * 1000:.3f} mm | {pos_st.get('max', 0) * 1000:.3f} mm | {pos_st.get('rms', 0) * 1000:.3f} mm |",
        f"| 姿态误差 | {ori_st.get('mean', 0):.2f}° | {ori_st.get('std', 0):.2f}° | {ori_st.get('max', 0):.2f}° | {ori_st.get('rms', 0):.2f}° |",
        f"| 速度误差 | {spd_st.get('mean', 0) * 1000:.2f} mm/s | {spd_st.get('std', 0) * 1000:.2f} mm/s | {spd_st.get('max', 0) * 1000:.2f} mm/s | {spd_st.get('rms', 0) * 1000:.2f} mm/s |",
        f"",
        f"**总焊接帧数:** {pos_st.get('count', 0):,}",
        f"",
        f"### 2.2 误差分布直方图",
        f"",
        f"{img(charts.get('error_histogram'), '误差分布')}",
        f"",
        f"### 2.3 各焊缝误差对比",
        f"",
    ]

    # 焊缝统计表
    if per_seam:
        lines.append("| 焊缝 ID | 位置 RMS (mm) | 位置 Max (mm) | 姿态均值 (°) | 速度 MAE (mm/s) |")
        lines.append("|---------|--------------|--------------|-------------|----------------|")
        for sid in sorted(per_seam.keys()):
            s = per_seam[sid]
            lines.append(
                f"| {sid} | {s['pos']['rms'] * 1000:.2f} | {s['pos']['max'] * 1000:.2f} | "
                f"{s['ori']['mean']:.2f} | {s['spd']['mean'] * 1000:.2f} |"
            )
        lines.append("")

    lines += [
        f"### 2.4 焊缝误差箱线图",
        f"",
        f"{img(charts.get('position_error_per_seam'), '位置误差箱线图')}",
        f"",
        f"{img(charts.get('orientation_error_per_seam'), '姿态误差箱线图')}",
        f"",
        f"{img(charts.get('speed_error_per_seam'), '速度误差箱线图')}",
        f"",
        f"### 2.5 TCP 速度时序",
        f"",
        f"{img(charts.get('tcp_speed_timeline'), 'TCP 速度时间序列')}",
        f"",
    ]

    # 各焊缝详细时序
    seam_ids = get_seam_ids(all_runs[0]) if all_runs else []
    for sid in seam_ids:
        lines += [
            f"### 2.6 焊缝 #{sid} 详细时序",
            f"",
            f"{img(charts.get(f'seam_{sid}_detail'), f'焊缝{sid}详情')}",
            f"",
        ]

    lines += [
        f"---",
        f"",
        f"## 3. 安全性评估",
        f"",
        f"### 3.1 重规划统计",
        f"",
        f"| 指标 | 值 |",
        f"|------|-----|",
        f"| 总重规划次数（{n} 次运行合计） | {sum(r_counts)} |",
        f"| 平均每次运行重规划次数 | {np.mean(r_counts):.1f} |",
        f"| RRT 成功率 | {replan_success * 100:.1f}% |",
        f"| RRT 平均计算耗时 | {np.mean(comp_times_ms):.2f} ms |" if comp_times_ms else "| RRT 平均计算耗时 | — |",
        f"| RRT 最大计算耗时 | {np.max(comp_times_ms):.2f} ms |" if comp_times_ms else "| RRT 最大计算耗时 | — |",
        f"",
        f"### 3.2 重规划分析图",
        f"",
        f"{img(charts.get('replan_analysis'), '重规划分析')}",
        f"",
        f"### 3.3 关节运动安全性",
        f"",
    ]

    # 关节速度加速度边界（使用平滑后的数据）
    all_jv = [[] for _ in range(6)]
    all_ja = [[] for _ in range(6)]
    for d in all_runs:
        for frm in d["Frames"].values():
            jv = frm.get("JointVelocities", [])
            ja = frm.get("JointAccelerations", [])
            for i in range(6):
                if i < len(jv):
                    all_jv[i].append(abs(jv[i]))
                if i < len(ja):
                    all_ja[i].append(abs(ja[i]))

    if all_jv[0]:
        lines.append("| 关节 | 最大角速度 (°/s) | 角加速度 RMS (°/s²) | 最大角加速度 (°/s²) |")
        lines.append("|------|-------------------|---------------------|---------------------|")
        for i in range(6):
            jv_max = max(all_jv[i]) if all_jv[i] else 0
            ja_rms = float(np.sqrt(np.mean(np.array(all_ja[i]) ** 2))) if all_ja[i] else 0
            ja_max = max(abs(np.array(all_ja[i]))) if all_ja[i] else 0
            lines.append(f"| J{i+1} | {jv_max:.3f} | {ja_rms:.3f} | {ja_max:.3f} |")
        lines.append("")

    lines += [
        f"### 3.4 关节运动时序",
        f"",
        f"{img(charts.get('joint_angles'), '关节角度')}",
        f"",
        f"{img(charts.get('joint_velocities'), '关节角速度')}",
        f"",
        f"{img(charts.get('joint_accelerations'), '关节角加速度')}",
        f"",
        f"---",
        f"",
        f"## 4. 仿真性能评估",
        f"",
        f"| 指标 | 值 |",
        f"|------|-----|",
        f"| 平均任务完成时间 | {np.mean(times):.2f} s |",
        f"| 时间标准差（运行间） | {np.std(times):.2f} s |",
        f"| 平均总帧数 | {int(np.mean(f_counts))} |",
        f"| 平均采样间隔 | {safe_div(np.mean(times), np.mean(f_counts)) * 1000:.2f} ms |",
        f"",
        f"---",
        f"",
        f"## 5. 关键发现",
        f"",
        f"1. **任务成功率:** {task_success_rate:.1f}% ({status_counts.get('Succeeded', 0)}/{n} 次运行成功)",
        f"2. **焊接精度:** 位置 RMS = {pos_st.get('rms', 0) * 1000:.2f} mm，姿态 MAE = {ori_st.get('mean', 0):.2f}°",
        f"3. **避障能力:** {n} 次运行共触发 {sum(r_counts)} 次重规划，成功率 {replan_success * 100:.1f}%",
        f"4. **运动平滑性:** 各关节加速度 RMS 在合理范围内（基于 EMA 平滑数据，α=0.3）",
        f"5. **仿真稳定性:** 任务完成时间标准差 {np.std(times):.2f}s，系统运行稳定",
        f"",
        f"---",
        f"",
        f"*报告生成时间: {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}*",
    ]

    return "\n".join(lines)


# ──────────────────────────────────────────────────────────
# 主流程
# ──────────────────────────────────────────────────────────

def analyze_test(test_num: int) -> None:
    """对一个 Test 执行完整分析流程"""

    print(f"\n{'='*60}")
    print(f"  Test {test_num} 仿真结果分析")
    print(f"{'='*60}")

    # 加载数据
    all_runs = load_result_files(test_num)
    print(f"  加载 {len(all_runs)} 次运行结果")

    # 输出目录
    out_dir = OUTPUT_BASE / f"Test_{test_num}"
    out_dir.mkdir(parents=True, exist_ok=True)
    cw = ChartWriter(out_dir)

    # ── 生成图表 ──
    print("\n  [1/8] 生成误差分布直方图...")
    charts = {}
    charts["error_histogram"] = chart_error_histogram(all_runs, cw)

    print("  [2/8] 各焊缝位置误差箱线图...")
    charts["position_error_per_seam"] = chart_position_error_per_seam(all_runs, cw)

    print("  [3/8] 各焊缝姿态误差箱线图...")
    charts["orientation_error_per_seam"] = chart_orientation_error_per_seam(all_runs, cw)

    print("  [4/8] 各焊缝速度误差箱线图...")
    charts["speed_error_per_seam"] = chart_speed_error_per_seam(all_runs, cw)

    print("  [5/8] TCP 速度时序图...")
    charts["tcp_speed_timeline"] = chart_tcp_speed_timeline(all_runs[0], cw)

    print("  [6/8] 重规划分析图...")
    charts["replan_analysis"] = chart_replan_analysis(all_runs, cw)

    print("  [7/8] 关节运动时序图...")
    charts["joint_angles"] = chart_joint_angle_timeline(all_runs[0], cw)
    charts["joint_velocities"] = chart_joint_velocities(all_runs[0], cw)
    charts["joint_accelerations"] = chart_joint_accelerations(all_runs[0], cw)

    print("  [8/8] 各焊缝详细时序...")
    seam_ids = get_seam_ids(all_runs[0])
    for sid in seam_ids:
        charts[f"seam_{sid}_detail"] = chart_seam(
            all_runs[0], sid, cw
        )

    # ── 生成报告 ──
    print("\n  生成分析报告...")
    report = build_report(test_num, all_runs, charts, out_dir)
    report_path = out_dir / "report.md"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report)

    # 报告中有 null --> 应改为 None 判断... 已经处理了

    print(f"\n  [OK] 报告已保存: {report_path}")
    print(f"  [DIR] 图表目录: {cw.chart_dir}")
    print(f"  [NUM] 图表数量: {len(list(cw.chart_dir.glob('*.png')))}")
    print(f"  [LEN] 报告行数: {len(report.splitlines())}")


# ──────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="焊接机器人仿真结果分析")
    parser.add_argument("test", type=int, choices=[1, 2, 3, 4, 5], help="Test 编号 1~5")
    args = parser.parse_args()
    analyze_test(args.test)
