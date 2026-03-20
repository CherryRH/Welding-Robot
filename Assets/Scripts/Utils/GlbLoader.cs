using GLTFast;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// GLB模型文件加载工具类
/// </summary>
public class GlbLoader : MonoBehaviour, IDisposable
{
    private GltfImport gltf;

    public async Task<GameObject> LoadAsync(string filePath, Transform parent = null)
    {
        Dispose(); // 清理之前的资源

        gltf = new GltfImport();
        string url = "file://" + filePath;

        try
        {
            // 加载文件
            if (!await gltf.Load(url))
            {
                Debug.LogError("加载失败: " + filePath);
                return null;
            }

            // 创建容器
            GameObject container = new(Path.GetFileNameWithoutExtension(filePath));
            if (parent != null)
                container.transform.SetParent(parent, false);

            // 实例化
            if (!await gltf.InstantiateMainSceneAsync(container.transform))
            {
                Destroy(container);
                return null;
            }

            return container;
        }
        catch (Exception e)
        {
            Debug.LogError($"加载异常: {e.Message}");
            return null;
        }
    }

    public void Dispose()
    {
        gltf?.Dispose();
        gltf = null;
    }

    private void OnDestroy()
    {
        Dispose();
    }
}
