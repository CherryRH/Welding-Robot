using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

/// <summary>
/// 显示 App 控制信息
/// </summary>
public class AppView : MonoBehaviour
{
    public TMP_Text SimulationTitleText;

    public void OnSimulationSwitche(int index)
    {
        SimulationTitleText.text = $"Simulation {index + 1}";
    }

    void Start()
    {
        
    }

    void Update()
    {
        
    }
}
