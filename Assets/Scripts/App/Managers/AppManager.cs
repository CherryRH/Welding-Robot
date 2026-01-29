using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// 应用程序管理器
/// </summary>
public class AppManager : MonoBehaviour
{
    // 仿真环境列表
    public SimulationContext SimulationPrefab;
    public List<SimulationContext> SimulationContexts = new();
    private SimulationContext currentContext;

    // UI组件
    public SimulationView SimulationView;
    public AppView AppView;
    public ControlView ControlView;

    // 控制管理
    public KeyInputManager KeyInputManager;

    // 回调事件
    public UnityEvent<int> OnSimulationSwitch;

    private void SwitchSimulation(int index)
    {
        // 切换当前 Simulation Context
        if (index < 0 || index >= SimulationContexts.Count) return;
        SimulationContext oldContext = currentContext;
        currentContext = SimulationContexts[index];

        // 绑定 context
        BindContextToApp(oldContext, currentContext);

        OnSimulationSwitch?.Invoke(index);

        Debug.Log($"Switch to Simulation {index}");
    }

    private void BindContextToApp(SimulationContext oldContext, SimulationContext newContext)
    {
        // 将 context 事件绑定到 App 组件上，同时移除旧绑定
        if (newContext == null) return;

        if (SimulationView != null)
        {
            if (oldContext != null) oldContext.OnSimulationUpdate.RemoveListener(SimulationView.OnSimulationUpdate);
            currentContext.OnSimulationUpdate.AddListener(SimulationView.OnSimulationUpdate);
        }

        if (ControlView != null)
        {
            if (oldContext != null)
            {
                oldContext.StateMachine.OnStateChange -= ControlView.OnStateChange;
                oldContext.OnClockUpdate.RemoveListener(ControlView.OnClockUpdate);
                oldContext.BeforeSimulationUpdate.RemoveListener(ControlView.BeforeSimulationUpdate);
            }
            currentContext.StateMachine.OnStateChange += ControlView.OnStateChange;
            currentContext.OnClockUpdate.AddListener(ControlView.OnClockUpdate);
            currentContext.BeforeSimulationUpdate.AddListener(ControlView.BeforeSimulationUpdate);
        }

        // 绑定键盘输入控制
        if (KeyInputManager != null)
        {
            if (oldContext != null) KeyInputManager.RemoveTarget(oldContext);
            KeyInputManager.AddTarget(currentContext);
        }
    }

    void Awake()
    {
        Debug.Log("Welcome to Welding Robot!");
        
        if (AppView != null)
        {
            OnSimulationSwitch.AddListener(AppView.OnSimulationSwitche);
        }
    }

    void Start()
    {
        SwitchSimulation(0);
    }

    void Update()
    {
        
    }

    void OnDestroy()
    {
        Debug.Log("Good bye!");
    }
}
