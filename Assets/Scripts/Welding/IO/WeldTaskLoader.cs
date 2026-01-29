using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 读取焊接任务数据
/// </summary>
public static class WeldTaskLoader
{
    public static WeldTask LoadFromJson(string json)
    {
        if (string.IsNullOrEmpty(json))
        {
            Debug.LogWarning("WeldTaskLoader.LoadFromJson: empty json");
            return new WeldTask();
        }

        try
        {
            var data = JsonUtil.Deserialize<WeldTask>(json) ?? new WeldTask();
            // 确保列表不为 null
            data.WeldSeams ??= new List<WeldSeam>();
            return data;
        }
        catch (Exception ex)
        {
            Debug.LogError($"WeldTaskLoader.LoadFromJson: failed to parse json. {ex}");
            return new WeldTask();
        }
    }

    /// <summary>
    /// 读取指定文件路径（绝对路径）的 WeldTask 文件并反序列化
    /// </summary>
    public static WeldTask LoadFromFile(string path)
    {
        if (string.IsNullOrEmpty(path))
        {
            Debug.LogWarning("WeldTaskLoader.LoadFromFile: path is null or empty.");
            return new WeldTask();
        }

        if (!File.Exists(path))
        {
            Debug.LogWarning($"WeldTaskLoader.LoadFromFile: file not found: {path}");
            return new WeldTask();
        }

        try
        {
            string json = File.ReadAllText(path);
            return LoadFromJson(json);
        }
        catch (Exception ex)
        {
            Debug.LogError($"WeldTaskLoader.LoadFromFile: failed to read file {path}. {ex}");
            return new WeldTask();
        }
    }
}
