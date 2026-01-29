using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Â·¾¶Æ¬¶Î
/// </summary>
public class TrajectorySegment
{
    public float StartTime;
    public float EndTime;

    public Pose StartTCP;
    public Pose EndTCP;

    public float[] QStart;
    public float[] QEnd;

    public PathInterpolation[] Interpolations;
}
