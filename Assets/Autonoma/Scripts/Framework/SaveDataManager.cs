/* 
Copyright 2023 Autonoma, Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:

    http://www.apache.org/licenses/LICENSE-2.0

The software is provided "AS IS", WITHOUT WARRANTY OF ANY KIND, 
express or implied. In no event shall the authors or copyright 
holders be liable for any claim, damages or other liability, 
whether in action of contract, tort or otherwise, arising from, 
out of or in connection with the software or the use of the software.
*/
using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Newtonsoft.Json.Linq;
using Newtonsoft.Json;
using Newtonsoft.Json.Serialization;
public static class SaveDataManager
{
    private static string scenarioFolderName = "Scenarios/";
    private static string vehSetupFolderName = "VehicleSetups/";
    private static string sensorSetFolderName = "SensorSets/";
    private const int currentVersion = 1;

    private static string GetSavePath()
    {
        string path = "";
        path = Environment.GetFolderPath(Environment.SpecialFolder.Personal);
        return Path.Combine(path, "AWSIM");
    }

    public static void SaveScenario(ScenarioObj obj)
    {
        obj.Version = currentVersion;
        string json = JsonConvert.SerializeObject(obj,Formatting.Indented);;
        string folderName = scenarioFolderName;
        string fullPath = Path.Combine(GetSavePath(), folderName);

        if(!Directory.Exists(fullPath))
        {    
            Directory.CreateDirectory(fullPath);
        }
        string fullPath2 = Path.Combine(fullPath, obj.Name + ".dat");

        WriteToFile(fullPath2,json);
    }
    
    public static void SaveVehicleSetup(VehSetup obj)
    {
        obj.Version = currentVersion;
        string json = JsonConvert.SerializeObject(obj,Formatting.Indented);;
        string folderName = vehSetupFolderName;
        string fullPath = Path.Combine(GetSavePath(), folderName);
        
        if(!Directory.Exists(fullPath))
        {    
            Directory.CreateDirectory(fullPath);
        }
        string fullPath2 = Path.Combine(fullPath, obj.Name + ".dat");

        WriteToFile(fullPath2,json);
    }

    public static void SaveSensorSet(SensorSet obj)
    {
        obj.Version = currentVersion;
        var settings = new JsonSerializerSettings
        {
            ReferenceLoopHandling = ReferenceLoopHandling.Ignore,
            ContractResolver = new DefaultContractResolver 
            { 
                IgnoreSerializableInterface = true 
            }
        };
        settings.Converters.Add(new SensorConverter());
        settings.Converters.Add(new SensorOptionsConverter());

        string json = JsonConvert.SerializeObject(obj, Formatting.Indented, settings);
        string folderName = sensorSetFolderName;
        string fullPath = Path.Combine(GetSavePath(), folderName);
        
        if(!Directory.Exists(fullPath))
        {    
            Directory.CreateDirectory(fullPath);
        }
        string fullPath2 = Path.Combine(fullPath, obj.Name + ".dat");

        WriteToFile(fullPath2,json);
    }
    
    public static void DeleteScenario(ScenarioObj obj)
    {
        string folderName = scenarioFolderName;
        string fullPath = Path.Combine(GetSavePath(), folderName);
        string filePath = Path.Combine(fullPath, obj.Name + ".dat");

        if (File.Exists(filePath))
        {
            File.Delete(filePath);
            Debug.Log(obj.Name + " deleted successfully.");
        }
        else
        {
            Debug.LogWarning(obj.Name + " not found.");
        }
    }

    public static void DeleteVehicleSetup(VehSetup obj)
    {
        string folderName = vehSetupFolderName;
        string fullPath = Path.Combine(GetSavePath(), folderName);
        string filePath = Path.Combine(fullPath, obj.Name + ".dat");

        if (File.Exists(filePath))
        {
            File.Delete(filePath);
            Debug.Log(obj.Name + " deleted successfully.");
        }
        else
        {
            Debug.LogWarning(obj.Name + " not found.");
        }
    }

    public static void DeleteSensorSet(SensorSet obj)
    {
        string folderName = sensorSetFolderName;
        string fullPath = Path.Combine(GetSavePath(), folderName);
        string filePath = Path.Combine(fullPath, obj.Name + ".dat");

        if (File.Exists(filePath))
        {
            File.Delete(filePath);
            Debug.Log(obj.Name + " deleted successfully.");
        }
        else
        {
            Debug.LogWarning(obj.Name + " not found.");
        }
    }

    public static List<ScenarioObj> LoadAllScenarios()
    {
        string folderName = scenarioFolderName;
        string path = Path.Combine(GetSavePath(), folderName);
        if(!Directory.Exists(path))
        {    
            Directory.CreateDirectory(path);
        }

        FileInfo[] files = new DirectoryInfo(@path).GetFiles("*.dat").OrderBy(f => f.CreationTime).ToArray();

        List<ScenarioObj> LoadedObjs = new List<ScenarioObj>();
        for  (int i = 0 ; i < files.Length; i++)  // iterate over 'files' array
        {
            string result = File.ReadAllText(files[i].FullName);
            ScenarioObj loadedObj = JsonConvert.DeserializeObject<ScenarioObj>(result);

            if(loadedObj.Version == currentVersion)
            {
                LoadedObjs.Add(loadedObj);
            }
        }

        return LoadedObjs;
    }
    public static List<VehSetup> LoadAllVehicleSetups()
    {   
        string folderName = vehSetupFolderName;
        string path = Path.Combine(GetSavePath(), folderName);
        if(!Directory.Exists(path))
        {    
            Directory.CreateDirectory(path);
        }

        FileInfo[] files = new DirectoryInfo(@path).GetFiles("*.dat").OrderBy(f => f.CreationTime).ToArray();

        List<VehSetup> LoadedObjs = new List<VehSetup>();
        for  (int i = 0 ; i < files.Length; i++)  // iterate over 'files' array
        {
            string result = File.ReadAllText(files[i].FullName);
            VehSetup loadedObj = JsonConvert.DeserializeObject<VehSetup>(result);

            if(loadedObj.Version == currentVersion)
            {
                LoadedObjs.Add(loadedObj);
            }
        }

        return LoadedObjs;
    }

    public static List<SensorSet> LoadAllSensorSets()
    {   
        string folderName = sensorSetFolderName;
        string path = Path.Combine(GetSavePath(), folderName);
        if(!Directory.Exists(path))
        {    
            Directory.CreateDirectory(path);
        }

        FileInfo[] files = new DirectoryInfo(@path).GetFiles("*.dat").OrderBy(f => f.CreationTime).ToArray();

        List<SensorSet> LoadedObjs = new List<SensorSet>();
        for  (int i = 0 ; i < files.Length; i++)  // iterate over 'files' array
        {
            string result = File.ReadAllText(files[i].FullName);

            var settings = new JsonSerializerSettings
            {
                ContractResolver = new DefaultContractResolver 
                { 
                    IgnoreSerializableInterface = true 
                }
            };
            settings.Converters.Add(new SensorConverter());
            settings.Converters.Add(new SensorOptionsConverter());
        
            SensorSet loadedObj = JsonConvert.DeserializeObject<SensorSet>(result, settings);

            if(loadedObj.Version == currentVersion)
            {
                LoadedObjs.Add(loadedObj);
            }
        }
        return LoadedObjs;
    }

    public static bool WriteToFile(string fullPath, string a_FileContents)
    {
        try
        {
            File.WriteAllText(fullPath, a_FileContents);
            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to write to {fullPath} with exception {e}");
            return false;
        }
    }
    public static bool LoadFromFile(string fullPath, out string result)
    {
        try
        {
            result = File.ReadAllText(fullPath);
            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to read from {fullPath} with exception {e}");
            result = "";
            return false;
        }
    }
}

public class SensorConverter : JsonConverter
{
    public override bool CanConvert(Type objectType)
    {
        return (objectType == typeof(ISensor));
    }

    public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
    {
        throw new NotImplementedException();
    }

    public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
    {
        var jObject = JObject.Load(reader);
        var sensorType = jObject["SensorType"].Value<string>();
        var parsedSensorType = (SensorType)Enum.Parse(typeof(SensorType), jObject["SensorType"].Value<string>(), true);

        ISensor sensor = parsedSensorType switch
        {
            SensorType.GNSS_INS => new Sensor<GnssInsSensorOptions>(parsedSensorType),
            SensorType.RAPTOR => new Sensor<RaptorSensorOptions>(parsedSensorType),
            SensorType.LIDAR => new Sensor<LidarSensorOptions>(parsedSensorType),
            SensorType.CAMERA => new Sensor<CameraSensorOptions>(parsedSensorType),
            SensorType.RADAR => new Sensor<RadarSensorOptions>(parsedSensorType),
            SensorType.GROUND_TRUTH => new Sensor<GroundTruthSensorOptions>(parsedSensorType),
            _ => throw new ArgumentException($"The SensorType '{sensorType}' is not supported.")
        };

        serializer.Populate(jObject.CreateReader(), sensor);

        return sensor;
    }
}

public class SensorOptionsConverter : JsonConverter
{
    public override bool CanConvert(Type objectType)
    {
        return (objectType == typeof(ISensorOptions));
    }

    public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
    {
        JObject jObject = JObject.Load(reader);

        // Here assuming that your JSON data includes a "Model" property to determine the options type
        var model = jObject["Model"].Value<string>();

        ISensorOptions sensorOptions;

        switch (model)
        {
            case "GENERIC":
                sensorOptions = new GenericSensorOptions();
                break;
            case "GENERIC_GNSS":
            case "GENERIC_IMU":
            case "PWRPAK7":
            case "VN310":
                sensorOptions = new GnssInsSensorOptions();
                break;
            case "RAPTOR":
                sensorOptions = new RaptorSensorOptions();
                break;
            case "LIDAR":
            case "LUMINAR":
            case "VELODYNE":
            case "OUSTER":
            case "INNOVIZ":
                sensorOptions = new LidarSensorOptions();
                break;
            case "CAMERA":
                sensorOptions = new CameraSensorOptions();
                break;
            case "RADAR":
                sensorOptions = new RadarSensorOptions();
                break;
            case "GROUND_TRUTH":
                sensorOptions = new GroundTruthSensorOptions();
                break;
            default:
                throw new Exception($"The given sensor options type '{model}' is not supported!");
        }

        serializer.Populate(jObject.CreateReader(), sensorOptions);

        return sensorOptions;
    }

    public override bool CanWrite => false;

    public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
    {
        throw new NotImplementedException("Unnecessary because CanWrite is false. The type will skip the converter.");
    }
}