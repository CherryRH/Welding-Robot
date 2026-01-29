using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// 键盘输入控制
/// </summary>
public class KeyInputManager : MonoBehaviour
{
    public UnityEvent<int> OnAlphaNumberKeyPressed;
    public UnityEvent<int> OnKeypadNumberKeyPressed;
    public UnityEvent<KeyCode> OnArrowKeyPressed;
    public UnityEvent<KeyCode> OnLetterKeyPressed;
    public UnityEvent<KeyCode> OnCommandKeyPressed;

    private readonly Dictionary<SimulationContext, RegisteredListeners> registrations = new();

    private class RegisteredListeners
    {
        public UnityAction<int> AlphaAction;
        public UnityAction<int> KeypadAction;
        public UnityAction<KeyCode> ArrowAction;
        public UnityAction<KeyCode> LetterAction;
        public UnityAction<KeyCode> CommandAction;
    }

    public void AddTarget(SimulationContext target)
    {
        if (target == null || registrations.ContainsKey(target)) return;

        var reg = new RegisteredListeners();

        reg.AlphaAction = num =>
        {
            switch (num)
            {
                case 1: target.TryChangeState(SimulationState.Idle); break;
                case 9: target.TryChangeState(SimulationState.Joint); break;
                case 0: target.TryChangeState(SimulationState.TCP); break;
            }
        };
        reg.KeypadAction = num =>
        {
            target.TryInput(KeyCode.None, num);
        };
        reg.ArrowAction = key =>
        {
            target.TryInput(key);
        };
        reg.LetterAction = key =>
        {
            switch (key)
            {
                case KeyCode.P:
                    LogUtil.PrintSimulationContextDebugLog(target);
                    break;
                case KeyCode.O:
                    LogUtil.SaveSimulationContextLogToFile(target);
                    break;
            }
        };
        reg.CommandAction = key =>
        {
            target.TryInput(key);
        };

        // 保存并绑定
        registrations[target] = reg;
        OnAlphaNumberKeyPressed.AddListener(reg.AlphaAction);
        OnKeypadNumberKeyPressed.AddListener(reg.KeypadAction);
        OnArrowKeyPressed.AddListener(reg.ArrowAction);
        OnLetterKeyPressed.AddListener(reg.LetterAction);
        OnCommandKeyPressed.AddListener(reg.CommandAction);
    }

    public void RemoveTarget(SimulationContext target)
    {
        if (target == null || !registrations.TryGetValue(target, out var reg)) return;

        OnAlphaNumberKeyPressed.RemoveListener(reg.AlphaAction);
        OnKeypadNumberKeyPressed.RemoveListener(reg.KeypadAction);
        OnArrowKeyPressed.RemoveListener(reg.ArrowAction);
        OnLetterKeyPressed.RemoveListener(reg.LetterAction);
        OnCommandKeyPressed.RemoveListener(reg.CommandAction);

        registrations.Remove(target);
    }

    void Start()
    {
        
    }

    void Update()
    {
        GetCommandKeyDown();
        GetAlphaNumberKeyDown();
        GetKeypadNumberKeyDown();
        GetArrowKey();
        GetLetterKeyDown();
    }

    private void GetAlphaNumberKeyDown()
    {
        // 检测主键盘区数字键按下
        int num = -1;
        if (Input.GetKeyDown(KeyCode.Alpha0)) num = 0;
        else if (Input.GetKeyDown(KeyCode.Alpha1)) num = 1;
        else if (Input.GetKeyDown(KeyCode.Alpha2)) num = 2;
        else if (Input.GetKeyDown(KeyCode.Alpha3)) num = 3;
        else if (Input.GetKeyDown(KeyCode.Alpha4)) num = 4;
        else if (Input.GetKeyDown(KeyCode.Alpha5)) num = 5;
        else if (Input.GetKeyDown(KeyCode.Alpha6)) num = 6;
        else if (Input.GetKeyDown(KeyCode.Alpha7)) num = 7;
        else if (Input.GetKeyDown(KeyCode.Alpha8)) num = 8;
        else if (Input.GetKeyDown(KeyCode.Alpha9)) num = 9;
        if (num > -1) OnAlphaNumberKeyPressed?.Invoke(num);
    }

    private void GetKeypadNumberKeyDown()
    {
        // 检测小键盘区数字键按下
        int num = -1;
        if (Input.GetKeyDown(KeyCode.Keypad0)) num = 0;
        else if (Input.GetKeyDown(KeyCode.Keypad1)) num = 1;
        else if (Input.GetKeyDown(KeyCode.Keypad2)) num = 2;
        else if (Input.GetKeyDown(KeyCode.Keypad3)) num = 3;
        else if (Input.GetKeyDown(KeyCode.Keypad4)) num = 4;
        else if (Input.GetKeyDown(KeyCode.Keypad5)) num = 5;
        else if (Input.GetKeyDown(KeyCode.Keypad6)) num = 6;
        else if (Input.GetKeyDown(KeyCode.Keypad7)) num = 7;
        else if (Input.GetKeyDown(KeyCode.Keypad8)) num = 8;
        else if (Input.GetKeyDown(KeyCode.Keypad9)) num = 9;
        if (num > -1) OnKeypadNumberKeyPressed?.Invoke(num);
    }

    private void GetArrowKey()
    {
        // 检测方向键按下
        if (Input.GetKey(KeyCode.UpArrow)) OnArrowKeyPressed?.Invoke(KeyCode.UpArrow);
        else if (Input.GetKey(KeyCode.DownArrow)) OnArrowKeyPressed?.Invoke(KeyCode.DownArrow);
        else if (Input.GetKey(KeyCode.LeftArrow)) OnArrowKeyPressed?.Invoke(KeyCode.LeftArrow);
        else if (Input.GetKey(KeyCode.RightArrow)) OnArrowKeyPressed?.Invoke(KeyCode.RightArrow);
    }

    private void GetLetterKeyDown()
    {
        // 检测字母键按下
        if (Input.GetKeyDown(KeyCode.P)) OnLetterKeyPressed?.Invoke(KeyCode.P);
        if (Input.GetKeyDown(KeyCode.O)) OnLetterKeyPressed?.Invoke(KeyCode.O);
    }

    private void GetCommandKeyDown()
    {
        // 检测命令键按下
        if (Input.GetKeyDown(KeyCode.LeftShift)) OnCommandKeyPressed?.Invoke(KeyCode.LeftShift);
        if (Input.GetKeyDown(KeyCode.RightShift)) OnCommandKeyPressed?.Invoke(KeyCode.RightShift);
        if (Input.GetKeyDown(KeyCode.Space)) OnCommandKeyPressed?.Invoke(KeyCode.Space);
    }
}
