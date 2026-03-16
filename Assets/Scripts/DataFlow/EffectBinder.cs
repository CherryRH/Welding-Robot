using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 特效绑定
/// </summary>
public class EffectBinder : MonoBehaviour
{
    // 焊接点特效
    public ParticleSystem ArcParticles;
    public ParticleSystem SparkParticles;
    public ParticleSystem CoolingParticles;

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void PlayWeldingEffect()
    {
        // 播放焊接特效
        if (ArcParticles != null && !ArcParticles.isPlaying) ArcParticles.Play();
        if (SparkParticles != null && !SparkParticles.isPlaying) SparkParticles.Play();
        if (CoolingParticles != null && !CoolingParticles.isPlaying) CoolingParticles.Play();
    }

    public void StopWeldingEffect()
    {
        // 停止焊接特效
        if (ArcParticles != null && ArcParticles.isPlaying) ArcParticles.Stop();
        if (SparkParticles != null && SparkParticles.isPlaying) SparkParticles.Stop();
        if (CoolingParticles != null && ArcParticles.isPlaying) CoolingParticles.Stop();
    }
}
