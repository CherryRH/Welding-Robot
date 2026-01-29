using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

/// <summary>
/// Json 工具（支持 UnityEngine.Vector3 与 WeldSeamData 的多态反序列化）
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
        settings.Converters.Add(new WeldSeamDataConverter());
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

    private class WeldSeamDataConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return typeof(WeldSeam).IsAssignableFrom(objectType);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
        {
            var jo = JObject.Load(reader);
            WeldSeam result = new();
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
