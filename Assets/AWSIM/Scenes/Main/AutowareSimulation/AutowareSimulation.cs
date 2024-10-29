using UnityEngine;
using System;
using AWSIM.TrafficSimulation;

namespace AWSIM
{
    /// <summary>
    /// Script for Scene. By Scirpt-Execution-Order, This class is called earlier than other MonoBehaviour.
    /// Enables configuration at scene startup.
    /// </summary>
    public class AutowareSimulation : MonoBehaviour
    {
        [SerializeField] TrafficManager trafficManager;
        [SerializeField] Transform egoTransform;
        [SerializeField] TimeSourceSelector timeSourceSelector;
        [SerializeField] VehicleG29Input vehicleG29Input;

        [Header("Player Config")]
        [SerializeField] string commandLineConfigParam = "--json_path";

        [Header("Editor Debug")]
        [SerializeField] bool useJsonConfig;

        [Tooltip("Specify this Path if you want to debug Json loading at Unity Editor runtime.")]
        [SerializeField] string jsonPath;

        [Serializable]
        public class EgoConfiguration
        {
            public Vector3 Position;
            public Vector3 EulerAngles;
        }

        [Serializable]
        public class ControllableNpcConfiguration
        {
            public string Name;
            public Vector3 Position;
            public Vector3 EulerAngles;
        }

        [Serializable]
        public class Configuration
        {
            public float TimeScale;                 // Reflected in Time.timeScale
            public string TimeSource;               // Reflected in TimeSourceSelector
            public int RandomTrafficSeed;           // Reflected in TrafficManager.seed
            public int MaxVehicleCount;             // Reflected in TrafficManager.maxVehicleCount
            public string G29DevicePath;            // Reflected in VehicleG29Input.DevicePath
            public EgoConfiguration Ego = new EgoConfiguration();
            public ControllableNpcConfiguration ControllableNpc = new ControllableNpcConfiguration();
        }

        void Awake()
        {
            // check if time source selector is present
            if (timeSourceSelector == null)
            {
                Debug.LogWarning("TimeSource: There is no TimeSourceSelector object assigned in the inspector. The default time source will be used.");
            }

#if !UNITY_EDITOR
            // initialize
            useJsonConfig = false;
            jsonPath = "";

            // For player, if path is not specified as a command line argument, json is not read.
            useJsonConfig = CommandLineUtility.GetCommandLineArg(out jsonPath, commandLineConfigParam);
#endif

            if (useJsonConfig)
            {
                var config = CommandLineUtility.LoadJsonFromPath<Configuration>(jsonPath);
                Time.timeScale = config.TimeScale;
                trafficManager.seed = config.RandomTrafficSeed;
                trafficManager.maxVehicleCount = config.MaxVehicleCount;

                var position = config.Ego.Position - Environment.Instance.MgrsOffsetPosition;
                egoTransform.position = ROS2Utility.RosToUnityPosition(position);

                var rotation = Quaternion.Euler(config.Ego.EulerAngles);
                egoTransform.rotation = ROS2Utility.RosToUnityRotation(rotation);

                var g29DevicePath = config.G29DevicePath;
                if (g29DevicePath != null)
                    vehicleG29Input.DevicePath = g29DevicePath;

                // set time source
                timeSourceSelector?.SetType(config.TimeSource);

                // if controllable NPC manager is on the scene initialize it with json data
                ControllableNPCVehicleManager controllableNpcManager = FindObjectOfType<ControllableNPCVehicleManager>();
                if(controllableNpcManager != null)
                {
                    var npcPosition = config.ControllableNpc.Position - Environment.Instance.MgrsOffsetPosition;
                    var npcRotation = Quaternion.Euler(config.ControllableNpc.EulerAngles);

                    controllableNpcManager.DoStart(
                        config.ControllableNpc.Name,
                        ROS2Utility.RosToUnityPosition(npcPosition),
                        ROS2Utility.RosToUnityRotation(npcRotation)
                    );
                }

            }
        }
    }
}