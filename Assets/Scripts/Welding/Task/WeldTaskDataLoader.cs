using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 读取焊接任务数据
/// </summary>
public static class WeldTaskDataLoader
{
    public static WeldTaskData LoadFromJson(string json)
    {
        if (string.IsNullOrEmpty(json))
        {
            Debug.LogWarning("WeldTaskDataLoader.LoadFromJson: empty json");
            return new WeldTaskData();
        }

        try
        {
            var data = JsonUtil.Deserialize<WeldTaskData>(json) ?? new WeldTaskData();
            // 确保列表不为 null
            data.WeldSeams ??= new List<WeldSeamData>();
            return data;
        }
        catch (Exception ex)
        {
            Debug.LogError($"WeldTaskDataLoader.LoadFromJson: failed to parse json. {ex}");
            return new WeldTaskData();
        }
    }

    /// <summary>
    /// 读取指定文件路径（绝对路径）的 WeldTask 文件并反序列化
    /// </summary>
    public static WeldTaskData LoadFromFile(string path)
    {
        if (string.IsNullOrEmpty(path))
        {
            Debug.LogWarning("WeldTaskDataLoader.LoadFromFile: path is null or empty.");
            return new WeldTaskData();
        }

        if (!File.Exists(path))
        {
            Debug.LogWarning($"WeldTaskDataLoader.LoadFromFile: file not found: {path}");
            return new WeldTaskData();
        }

        try
        {
            string json = File.ReadAllText(path);
            return LoadFromJson(json);
        }
        catch (Exception ex)
        {
            Debug.LogError($"WeldTaskDataLoader.LoadFromFile: failed to read file {path}. {ex}");
            return new WeldTaskData();
        }
    }
}
