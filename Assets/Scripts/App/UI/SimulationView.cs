using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

/// <summary>
/// 在UI中显示模拟仿真状态
/// </summary>
public class SimulationView : MonoBehaviour
{
    public TMP_Text[] JointAngleText = new TMP_Text[6];
    public TMP_Text[] JointPosText = new TMP_Text[18];
    public TMP_Text[] TCPPosText = new TMP_Text[3];
    public TMP_Text[] ToolEulerAnglesText = new TMP_Text[3];

    public void OnSimulationUpdate(SimulationContext context)
    {
        // 更新 UI（全部展示数据层数据）
        RobotModel robotModel = context.RobotModel;

        for (int i = 0; i < robotModel.JointsCount; i++)
        {
            JointAngleText[i].text = robotModel.Joints[i].Angle.ToString("F0");

            Vector3 jointPos = robotModel.Joints[i].Position;
            JointPosText[3 * i].text = jointPos.x.ToString("F2");
            JointPosText[3 * i + 1].text = jointPos.y.ToString("F2");
            JointPosText[3 * i + 2].text = jointPos.z.ToString("F2");
        }

        Vector3 tcpPos = robotModel.TCPPosition;
        TCPPosText[0].text = tcpPos.x.ToString("F2");
        TCPPosText[1].text = tcpPos.y.ToString("F2");
        TCPPosText[2].text = tcpPos.z.ToString("F2");

        Vector3 toolEuler = robotModel.ToolEularAngles;
        ToolEulerAnglesText[0].text = toolEuler.x.ToString("F0");
        ToolEulerAnglesText[1].text = toolEuler.y.ToString("F0");
        ToolEulerAnglesText[2].text = toolEuler.z.ToString("F0");
    }

    void Start()
    {

    }

    void Update()
    {
        
    }
}
