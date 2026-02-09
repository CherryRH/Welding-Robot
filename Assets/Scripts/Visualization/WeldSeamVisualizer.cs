using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.UI.Image;

/// <summary>
/// º¸·ì¿ÉÊÓ»¯Æ÷
/// </summary>
public class WeldSeamVisualizer : MonoBehaviour
{
    public Transform WorkpieceOrigin;

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void ShowSeams(WeldTask task, float duration)
    {
        foreach (var seam in task.WeldSeams)
        {
            int segments = 50;
            Vector3 prev = seam.GetPoint(0f);

            for (int i = 1; i <= segments; i++)
            {
                float s = (float)i / segments;
                Vector3 cur = seam.GetPoint(s);

                Vector3 wp0 = WorkpieceOrigin.position + MathUtil.DataToUnityPosition(prev);
                Vector3 wp1 = WorkpieceOrigin.position + MathUtil.DataToUnityPosition(cur);

                Debug.DrawLine(wp0, wp1, Color.cyan, duration);

                prev = cur;
            }
        }
    }
}
