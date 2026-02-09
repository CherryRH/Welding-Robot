using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

/// <summary>
/// Json 工具
/// </summary>
public static class JsonUtil
{
    private static JsonSerializerSettings CreateSettings()
    {
        var settings = new JsonSerializerSettings
        {
            Formatting = Formatting.Indented,
            NullValueHandling = NullValueHandling.Ignore
        };
        settings.Converters.Add(new Vector3Converter());
        settings.Converters.Add(new WeldSeamConverter());
        settings.Converters.Add(new PoseConverter());
        return settings;
    }

    public static string Serialize(object obj)
    {
        return JsonConvert.SerializeObject(obj, CreateSettings());
    }

    public static T Deserialize<T>(string json)
    {
        return JsonConvert.DeserializeObject<T>(json, CreateSettings());
    }

    private class Vector3Converter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(Vector3);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
        {
            if (reader.TokenType == JsonToken.StartObject)
            {
                JObject jo = JObject.Load(reader);
                float x = jo["x"]?.Value<float>() ?? 0f;
                float y = jo["y"]?.Value<float>() ?? 0f;
                float z = jo["z"]?.Value<float>() ?? 0f;
                return new Vector3(x, y, z);
            }
            else if (reader.TokenType == JsonToken.StartArray)
            {
                JArray ja = JArray.Load(reader);
                float x = ja.Count > 0 ? ja[0].Value<float>() : 0f;
                float y = ja.Count > 1 ? ja[1].Value<float>() : 0f;
                float z = ja.Count > 2 ? ja[2].Value<float>() : 0f;
                return new Vector3(x, y, z);
            }

            return Vector3.zero;
        }

        public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
        {
            var v = (Vector3)value;
            writer.WriteStartObject();
            writer.WritePropertyName("x"); writer.WriteValue(v.x);
            writer.WritePropertyName("y"); writer.WriteValue(v.y);
            writer.WritePropertyName("z"); writer.WriteValue(v.z);
            writer.WriteEndObject();
        }
    }

    private class PoseConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(Pose);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
        {
            if (reader.TokenType == JsonToken.StartObject)
            {
                JObject jo = JObject.Load(reader);

                // 读取位置信息
                float x = jo["x"]?.Value<float>() ?? 0f;
                float y = jo["y"]?.Value<float>() ?? 0f;
                float z = jo["z"]?.Value<float>() ?? 0f;
                Vector3 position = new(x, y, z);

                // 优先尝试读取四元数
                if (jo["qw"] != null && jo["qx"] != null && jo["qy"] != null && jo["qz"] != null)
                {
                    float qx = jo["qx"].Value<float>();
                    float qy = jo["qy"].Value<float>();
                    float qz = jo["qz"].Value<float>();
                    float qw = jo["qw"].Value<float>();
                    Quaternion rotation = new(qx, qy, qz, qw);
                    return new Pose(position, rotation);
                }
                // 如果存在欧拉角，则转换为四元数
                else if (jo["roll"] != null && jo["pitch"] != null && jo["yaw"] != null)
                {
                    float eulerX = jo["roll"].Value<float>();
                    float eulerY = jo["pitch"].Value<float>();
                    float eulerZ = jo["yaw"].Value<float>();
                    Quaternion rotation = Quaternion.Euler(eulerX, eulerY, eulerZ);
                    return new Pose(position, rotation);
                }
                // 默认使用单位四元数
                else
                {
                    return new Pose(position, Quaternion.identity);
                }
            }

            return new Pose(Vector3.zero, Quaternion.identity);
        }

        public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
        {
            var v = (Pose)value;
            Vector3 position = v.position;
            Quaternion rotation = v.rotation;

            Vector3 eulerAngles = rotation.eulerAngles;

            writer.WriteStartObject();
            writer.WritePropertyName("x"); writer.WriteValue(position.x);
            writer.WritePropertyName("y"); writer.WriteValue(position.y);
            writer.WritePropertyName("z"); writer.WriteValue(position.z);
            writer.WritePropertyName("roll"); writer.WriteValue(eulerAngles.x);
            writer.WritePropertyName("pitch"); writer.WriteValue(eulerAngles.y);
            writer.WritePropertyName("yaw"); writer.WriteValue(eulerAngles.z);
            writer.WritePropertyName("qx"); writer.WriteValue(rotation.x);
            writer.WritePropertyName("qy"); writer.WriteValue(rotation.y);
            writer.WritePropertyName("qz"); writer.WriteValue(rotation.z);
            writer.WritePropertyName("qw"); writer.WriteValue(rotation.w);
            writer.WriteEndObject();
        }
    }

    private class WeldSeamConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return typeof(WeldSeamData).IsAssignableFrom(objectType);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
        {
            var jo = JObject.Load(reader);
            WeldSeamData result = new();
            // 将 JObject 的内容填充到具体类型实例
            serializer.Populate(jo.CreateReader(), result);
            return result;
        }

        public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
        {
            // 采用默认序列化（包含嵌套字段）
            JObject jo = JObject.FromObject(value, JsonSerializer.Create(CreateSettings()));
            jo.WriteTo(writer);
        }
    }
}
