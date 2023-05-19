using System.Collections;
using System;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

namespace AWSIM.Loader 
{
    public enum LogLevel
    {
        LOG_INFO = 0,
        LOG_ERROR = 1
    }

    /// <summary>
    /// This class is the main entry point for loading the simulation.
    /// It is responsible for loading the JSON configuration file or showing the GUI configuration window if no configuration file is available.
    ///
    /// Configuration file path can be provided by using the CLI argument:
    /// --config <FILE>
    ///
    /// It is also possible to let it look for the file automatically inside the binary "_Data" directory or Editor's "Assets".
    ///
    /// The loader composes the simulation with the "AWSIMSimulation" and "Map" scenes, which are loaded additively.
    /// AWSIMSimulation is a fixed scene, which contains core simulation GameObjects like a clock, RGL, and Ros2ForUnity libraries, as well as in-simulation GUI,
    /// A map scene is any scene that contains the environment representation of the simulation. It may be any scene included in the project's build list.
    ///
    /// After loading the necessary scenes (AWSIMSimulation and chosen map), the scene is configured via the configuration managers.
    ///
    /// Loader class persists during the simulation and can be used to reload maps in runtime.
    /// </summary>
    public class Loader : MonoBehaviour
    {
        [Header("Parameter file")]
        [SerializeField]
        [Tooltip("Should the loader look for a configuration file in '_Data' or 'Assets' directory.")]
        private bool useLocalFile = false;
        [SerializeField]
        [Tooltip("CLI arg to fetch the configuration file path.")]
        private string configParam = "--config";
        [SerializeField]
        [Tooltip("If the local file is used, it looks for a file tiwh default name.")]
        private string defaultConfigFilename = "config.json";

        [Header("Managers")]
        [SerializeField]
        [Tooltip("Manager for setting and configuring the Ego vehicle.")]
        private EgosManager egoManager;
        [SerializeField]
        [Tooltip("Manager for setting and configuring the map environment.")]
        private MapManager mapManager;
        [SerializeField]
        [Tooltip("Manager for setting and configuring the simulation.")]
        private SimulationManager simulationManager;

        [Header("Loader GUI")]
        [SerializeField]
        [Tooltip("Configuration GUI root object.")]
        private GameObject rootGuiObject;

        [SerializeField]
        [Tooltip("JSON content preview text area.")]
        private Text jsonConfiguration;

        [SerializeField]
        [Tooltip("Log view area.")]
        private Text logField;

        [SerializeField]
        [Tooltip("JSON view canvas.")]
        private GameObject jsonCanvas;

        [Tooltip("Manual configuration canvas.")]
        [SerializeField]
        private GameObject manualCanvas;

        [Tooltip("Button for accepting the configuration and start loading.")]
        [SerializeField]
        private GameObject loadButton;

        [Tooltip("Loading screen displayed while the simulation is loading.")]
        [SerializeField]
        private GameObject loadingScreen;

        bool usingConfigFile = false;
        bool configFileLoaded = false;

        IEnumerator ReLoadCoroutine()
        {
            rootGuiObject.SetActive(true);

            // Show loading screen
            loadingScreen.gameObject.GetComponentInChildren<Text>().text = "";
            loadingScreen.SetActive(true);
            yield return new WaitForEndOfFrame();

            AsyncOperation reloadSceneOperation = simulationManager.ResetLoader();
            yield return new WaitUntil(() => reloadSceneOperation.isDone);
            
            // Hide loading screen and gui
            loadingScreen.SetActive(false);
            rootGuiObject.SetActive(false);
        }

        IEnumerator LoadCoroutine()
        {
            // Make sure loader gui is active
            rootGuiObject.SetActive(true);

            // Show loading screen
            loadingScreen.gameObject.GetComponentInChildren<Text>().text = $"Loading map '{mapManager.MapConfiguration.mapName}'...";
            loadingScreen.SetActive(true);
            yield return new WaitForEndOfFrame();

            // Load map
            AsyncOperation sceneLoad = mapManager.LoadMap();
            sceneLoad.allowSceneActivation = true;
            yield return new WaitUntil(() => sceneLoad.isDone);

            // Then load simulation components
            AsyncOperation simulationLoad = simulationManager.LoadSimulation();
            yield return new WaitUntil(() => simulationLoad.isDone);

            // Finally configure the scene
            ConfigureScenes();

            // Hide loading screen and gui
            loadingScreen.SetActive(false);
            rootGuiObject.SetActive(false);
            yield return null; 
        }

        public void Start()
        {
            // Turn off GUI canvases.
            jsonCanvas.SetActive(false);
            manualCanvas.SetActive(false);

            // Hook log actions for managers.
            mapManager.Log += Log;
            egoManager.Log += Log;
            simulationManager.Log += Log;

            StartLoader();
        }

        public void Update()
        {
            // Restart the whole simulation with keyboard key.
            // The configuration file is read again.
            if(Input.GetKeyUp(KeyCode.F12))
            {
                // Reload scene
                StartCoroutine(ReLoadCoroutine());
            }
        }

        /// <summary>
        /// Main loader function for detecting if the configuration file is available.
        /// If the file is available - compose scenes and start configuration
        /// If the file is not available - initialize GUI and wait for manual start
        /// </summary>
        public void StartLoader()
        {
            // Load from config file.
            // Firstly, get the config file path.
            if(GetConfigFilePath(out string path))
            {
                Log(LogLevel.LOG_INFO, $"Configuration file found. Loading configure file: {path}");
                jsonCanvas.SetActive(true);

                // Try to load the config file
                var configuration = LoadConfigFromFile(path);
                if (configuration != null)
                {
                    Log(LogLevel.LOG_INFO, "Configuration file loaded.");

                    // Configure managers with configuration file.
                    if (ConfigureManagers(configuration))
                    {
                        // Configuration went well. Load all scenes.
                        Load();
                    }
                } else {
                    // There is an error loading config file. Block the Load button.
                    loadButton.SetActive(false);
                }
            } else { // If config file is not found, fallback to GUI.
                Log(LogLevel.LOG_INFO, "No configuration file provided.");
                manualCanvas.SetActive(true);

                // Initialize UI with the manager resources. 
                mapManager.LoadUI();
                egoManager.LoadUI();
                simulationManager.LoadUI();
            }
        }

        /// <summary>
        /// Button callback for loading the simulation with the GUI configuration.
        /// </summary>
        public void OnLoadButtonPressed()
        {
            var configuration = LoadConfigFromGUI();

            if (ConfigureManagers(configuration))
            {
                Load();
            }
        }

        AWSIMConfiguration LoadConfigFromGUI()
        {
            if (!usingConfigFile)
            {
                AWSIMConfiguration simulationConfig = new AWSIMConfiguration();
                simulationConfig.mapConfiguration.mapName = mapManager.mapUISelecor.options[mapManager.mapUISelecor.value].text;
                simulationConfig.simulationConfiguration.useTraffic = simulationManager.mapTrafficToggle.isOn;
                simulationConfig.simulationConfiguration.timeScale = 1.0f; //TODO: Time scale is not yet implemented on the GUI.
                simulationConfig.egoConfiguration.egoVehicleName = egoManager.egoUISelecor.options[egoManager.egoUISelecor.value].text;
                simulationConfig.egoConfiguration.egoPosition = egoManager.GetPositionFromUI();
                simulationConfig.egoConfiguration.egoEulerAngles = egoManager.GetEulersFromUI();
                return simulationConfig;
            }
            return null;
        }

        AWSIMConfiguration LoadConfigFromFile(string path)
        {
            usingConfigFile = true;
            try
            {
                string content = File.ReadAllText(path);
                jsonConfiguration.text = content;
                AWSIMConfiguration simulationConfig = JsonUtility.FromJson<AWSIMConfiguration>(content);
                configFileLoaded = true;
                return simulationConfig;
            }
            catch (ArgumentException exception)
            {
                Log(LogLevel.LOG_ERROR, exception.Message);
                return null;
            }
            catch (FileNotFoundException exception)
            {
                Log(LogLevel.LOG_ERROR, exception.Message);
                return null;
            }
        }

        bool ConfigureManagers(AWSIMConfiguration awsimConfig)
        {
            if (usingConfigFile && !configFileLoaded)
            {
                Log(LogLevel.LOG_ERROR, "Can't configure simulation. Invalid configuration.");
                loadButton.SetActive(false);
                return false;
            }

            // Load config from json
            if (!egoManager.LoadConfig(awsimConfig)) 
            {
                if (usingConfigFile) loadButton.SetActive(false);
                return false;
            };
            if (!mapManager.LoadConfig(awsimConfig))
            {
                if (usingConfigFile) loadButton.SetActive(false);
                return false;
            };
            if (!simulationManager.LoadConfig(awsimConfig))
            {
                if (usingConfigFile) loadButton.SetActive(false);
                return false;
            }

            return true;
        }

        string GetCommandLineArg(string name)
        {
            var cmdArgs = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < cmdArgs.Length; i++)
            {
                if (cmdArgs[i] == name && cmdArgs.Length > i + 1)
                {
                    Debug.Log($"Cmd arg: {cmdArgs[i]} {cmdArgs[i + 1]}");
                    return cmdArgs[i + 1];
                }
            }
            return null;
        }

        bool GetConfigFilePath(out string path)
        {
            var configPathArg = GetCommandLineArg(configParam);
            if (configPathArg != null)
            {
                path = configPathArg;
                return true;
            } else if (useLocalFile) {
                char separator = Path.DirectorySeparatorChar;
                string appPath = Application.dataPath;
                path = appPath + separator + defaultConfigFilename;
                return true;
            }
            path = "";
            return false;
        }

        void Load()
        {
            StartCoroutine(LoadCoroutine());
        }

        void ConfigureScenes()
        {
            Scene simulationScene = SceneManager.GetSceneByName(simulationManager.simulationSceneName);
            Scene mapScene = SceneManager.GetSceneByName(mapManager.spawnedMapName);

            egoManager.ConfigureScene(simulationScene.GetRootGameObjects()[0].transform);
            mapManager.ConfigureScene(mapScene.GetRootGameObjects()[0].transform);
            simulationManager.ConfigureScene();
        }

        void Log(LogLevel level, string message)
        {
            switch (level)
            {
                case LogLevel.LOG_INFO:
                {
                    var msg = $"<color=\"#1F2933\">{message}</color>" + System.Environment.NewLine;
                    logField.text += msg;
                    Debug.Log(message);
                    break;
                }
                case LogLevel.LOG_ERROR:
                {
                    var msg = $"<color=\"red\">{message}</color>" + System.Environment.NewLine;
                    logField.text += msg;
                    Debug.LogError(message);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
}
