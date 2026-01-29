using Newtonsoft.Json.Linq;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

public static class WeldInstructionWriter
{
    public static string SaveToFile(SimulationContext context = null)
    {
        context ??= UnityEngine.Object.FindObjectOfType<SimulationContext>();
        if (context == null)
        {
            Debug.LogWarning("WeldInstructionWriter: SimulationContext not found.");
            return null;
        }

        try
        {
            string fileName = $"{context.name}_WeldInstruction_{DateTime.Now:yyyyMMdd_HHmmss}.json";
            string path = Path.Combine(Application.persistentDataPath, fileName);

            JObject jo = new()
            {
                ["WeldTaskName"] = context.WeldTask.TaskName,
                ["WorkpieceFileName"] = context.WeldTask.WorkpieceFileName,
            };
            JArray jArr = new();
            foreach (var item in context.WeldPlanner.Instructions)
            {
                jArr.Add(new JObject
                {
                    ["Type"] = item.Type.ToString(),
                    ["TargetPose"] = new JObject
                    {
                        ["x"] = item.TargetPose.position.x.ToString("F3"),
                        ["y"] = item.TargetPose.position.y.ToString("F3"),
                        ["z"] = item.TargetPose.position.z.ToString("F3"),
                        ["roll"] = item.TargetPose.rotation.eulerAngles.x.ToString("F1"),
                        ["pitch"] = item.TargetPose.rotation.eulerAngles.y.ToString("F1"),
                        ["yaw"] = item.TargetPose.rotation.eulerAngles.z.ToString("F1"),
                        ["qx"] = item.TargetPose.rotation.x.ToString("F3"),
                        ["qy"] = item.TargetPose.rotation.y.ToString("F3"),
                        ["qz"] = item.TargetPose.rotation.z.ToString("F3"),
                        ["qw"] = item.TargetPose.rotation.w.ToString("F3"),
                    },
                    ["TargetJoints"] = item.TargetJoints != null && item.TargetJoints.Length >= 6 ? new JObject
                    {
                        ["J1"] = item.TargetJoints[0].ToString("F1"),
                        ["J2"] = item.TargetJoints[1].ToString("F1"),
                        ["J3"] = item.TargetJoints[2].ToString("F1"),
                        ["J4"] = item.TargetJoints[3].ToString("F1"),
                        ["J5"] = item.TargetJoints[4].ToString("F1"),
                        ["J6"] = item.TargetJoints[5].ToString("F1"),
                    } : new JObject(),
                    ["Speed"] = item.Speed,
                    ["IsContinous"] = item.IsContinous,
                    ["PauseDuration"] = item.PauseDuration
                });
            }
            jo.Add("Instructions", jArr);

            File.WriteAllText(path, jo.ToString(), Encoding.UTF8);
            Debug.Log($"WeldInstruction saved to: {path}");
            return path;
        }
        catch (Exception ex)
        {
            Debug.LogError($"Write WeldInstruction error: {ex}");
            return null;
        }
    }
}
