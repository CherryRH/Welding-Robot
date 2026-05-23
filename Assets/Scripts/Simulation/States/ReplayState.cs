using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 重播状态：回放已记录的仿真帧数据
/// </summary>
public class ReplayState : SimulationStateBase
{
    /// <summary>当前播放的帧索引（时间戳列表）</summary>
    private List<float> frameTimestamps = new();

    /// <summary>当前播放位置</summary>
    private int currentFrameIndex = 0;

    /// <summary>缓存的结果数据引用</summary>
    private WeldResultData resultData;

    public ReplayState(SimulationStateMachine m) : base(m) { }

    public override void Enter(SimulationContext ctx)
    {
        // 获取已记录的结果数据
        resultData = ctx.ResultWriter.GetResultData();

        if (resultData == null || resultData.Frames.Count == 0)
        {
            Debug.LogWarning("[Replay] 无可回放的帧数据，返回 Idle");
            ctx.TryChangeState(SimulationState.Idle);
            return;
        }

        // 提取时间戳列表
        frameTimestamps.Clear();
        foreach (var kvp in resultData.Frames)
            frameTimestamps.Add(kvp.Key);

        currentFrameIndex = 0;

        // 显示路径可视化（用 WeldPointData）
        if (ctx.TcpPathVisualizer != null && resultData.Points.Count > 0)
            ctx.TcpPathVisualizer.ShowWeldPointData(resultData.Points);

        // 启动时钟（从头开始）
        ctx.Clock.Reset();
        ctx.Clock.Start();

        Debug.Log($"[Replay] 开始回放 {frameTimestamps.Count} 帧");
    }

    public override void Update(SimulationContext ctx, float dt)
    {
        if (!ctx.Clock.IsRunning || currentFrameIndex >= frameTimestamps.Count)
        {
            // 播放完成，停在最后一帧
            if (currentFrameIndex >= frameTimestamps.Count && ctx.Clock.IsRunning)
            {
                ctx.Clock.Stop();
                Debug.Log("[Replay] 播放完成");
            }
            return;
        }

        // 获取当前帧
        float timestamp = frameTimestamps[currentFrameIndex];
        if (resultData.Frames.TryGetValue(timestamp, out WeldResultFrame frame))
        {
            // 应用关节角
            if (frame.JointAngles != null && frame.JointAngles.Length > 0)
                ctx.RobotModel.SetJointAngles(frame.JointAngles);

            // 根据帧类型控制焊接特效
            if (frame.PointType == WeldStateType.Weld)
            {
                ctx.EffectBinder.PlayWeldingEffect();
                if (ctx.MoltenPoolVisualizer != null && !ctx.MoltenPoolVisualizer.IsStripActive)
                    ctx.MoltenPoolVisualizer.StartStrip();
            }
            else
            {
                ctx.EffectBinder.StopWeldingEffect();
                if (ctx.MoltenPoolVisualizer != null && ctx.MoltenPoolVisualizer.IsStripActive)
                    ctx.MoltenPoolVisualizer.FinalizeStrip();
            }

            // 熔池采样
            if (ctx.MoltenPoolVisualizer != null)
                ctx.MoltenPoolVisualizer.AddSample(
                    ctx.RobotBinder.Tcp,
                    frame.TcpSpeed);

            // 推进帧索引
            currentFrameIndex++;
        }
    }

    public override void Exit(SimulationContext ctx)
    {
        ctx.EffectBinder.StopWeldingEffect();
        if (ctx.MoltenPoolVisualizer != null && ctx.MoltenPoolVisualizer.IsStripActive)
            ctx.MoltenPoolVisualizer.FinalizeStrip();
        ctx.Clock.Stop();
    }

    public override void HandleInput(SimulationContext ctx, KeyCode key, int num)
    {
        if (key == KeyCode.Space)
        {
            // 暂停/继续
            if (ctx.Clock.IsRunning) ctx.Clock.Stop();
            else ctx.Clock.Start();
        }
        else if (key == KeyCode.Escape)
        {
            // 退出到 Idle
            ctx.TryChangeState(SimulationState.Idle);
            ctx.Reset();
        }
        else if (key == KeyCode.Tab)
        {
            // 从头重新播放
            currentFrameIndex = 0;
            // 清空焊缝
            if (ctx.MoltenPoolVisualizer != null) ctx.MoltenPoolVisualizer.Clear();

            ctx.Clock.Reset();
            ctx.Clock.Start();
            Debug.Log("[Replay] 重新播放");
        }
    }
}
