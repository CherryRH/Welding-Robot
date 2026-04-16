using TMPro;
using UnityEngine;

/// <summary>
/// 控制面板 UI
/// </summary>
public class ControlView : MonoBehaviour
{
    public TMP_Text StateText;
    public TMP_Text IKText;
    public TMP_Text TimeText;
    public TMP_Text StateDetailText;

    public void BeforeSimulationUpdate(SimulationContext context)
    {
        SimulationStateBase stateBase = context.StateMachine.GetCurrentStateInstance();
        string text = "";
        if (stateBase is JointState jointState)
        {
            text += $"Joint {jointState.ControlledJoint + 1}, ";
            text += jointState.Symbol switch
            {
                DataChangeSymbol.Increase => "Increase",
                DataChangeSymbol.Decrease => "Decrease",
                _ => "Stay"
            };
        }
        else if (stateBase is TCPState tcpState)
        {
            text += $"TCP ";
            text += tcpState.ControlledData switch
            {
                1 => "X",
                2 => "Y",
                3 => "Z",
                4 => "Roll",
                5 => "Pitch",
                6 => "Yaw",
                _ => "Unknown"
            };
            text += ", ";
            text += tcpState.Symbol switch
            {
                DataChangeSymbol.Increase => "Increase",
                DataChangeSymbol.Decrease => "Decrease",
                _ => "Stay"
            };
        }
        else if (stateBase is WorkState workState)
        {
            if (context.Task != null)
            {
                text += $"Task: {context.Task.TaskName}\n";
            }
        }
        else if (stateBase is ReplayState replayState)
        {
            var resultData = context.ResultWriter.GetResultData();
            if (resultData != null)
            {
                text += $"Task: {resultData.TaskName}\n";
                text += $"Status: {resultData.PlanStatus}";
            }
        }

        StateDetailText.text = text;

        switch (context.RobotModel.IK.IKMethod)
        {
            case IK.IKMethodType.CCD:
                IKText.text = "CCD";
                break;
            case IK.IKMethodType.JT:
                IKText.text = "JT";
                break;
            case IK.IKMethodType.ANALYTIC:
                IKText.text = "ANALYTIC";
                break;
        }
    }

    public void OnStateChange(SimulationState from, SimulationState to)
    {
        switch (to)
        {
            case SimulationState.Idle:
                StateText.text = "Idle";
                break;
            case SimulationState.Work:
                StateText.text = "Work";
                break;
            case SimulationState.Succeed:
                StateText.text = "Succeed";
                break;
            case SimulationState.Fail:
                StateText.text = "Fail";
                break;
            case SimulationState.Replay:
                StateText.text = "Replay";
                break;
            case SimulationState.Joint:
                StateText.text = "Rotate Joint";
                break;
            case SimulationState.TCP:
                StateText.text = "Translate TCP";
                break;
        }
    }

    public void OnClockUpdate(SimulationClock clock)
    {
        TimeText.text = clock.Time.ToString("F2");
    }

    void Start()
    {

    }

    void Update()
    {

    }
}
