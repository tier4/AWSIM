using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Linq;


namespace AWSIM.Loader 
{
    /// <summary>
    /// Manager for configuring the Ego.
    /// </summary>
    public class MapManager : MonoBehaviour, IConfigurableManager
    {
        [Tooltip("Scene names that can't be used and aren't shown in a GUI selector.")]
        public List<string> forbiddenSceneNames = new List<string>(){"AWSIMSimulation"};
        
        [HideInInspector]
        public string spawnedMapName;
        
        private MapConfiguration mapConfiguration;

        /// <summary>
        /// Map configuration.
        /// </summary>
        public MapConfiguration MapConfiguration => mapConfiguration;
        
        [Tooltip("Map dropdown selector.")]
        public Dropdown mapUISelecor;
        
        /// <summary>
        /// Log callback.
        /// </summary>
        public Action<LogLevel, string> Log { get; set; }

        public void Start()
        {
            if (!forbiddenSceneNames.Contains(SceneManager.GetActiveScene().name))
            {
                forbiddenSceneNames.Add(SceneManager.GetActiveScene().name);
            }
        }

        /// <summary>
        /// Additively loads a map specified in a configuration.
        /// </summary>
        public AsyncOperation LoadMap()
        {
            Log(LogLevel.LOG_INFO, $"Loading scene {mapConfiguration.mapName}");
            spawnedMapName = mapConfiguration.mapName;
            return SceneManager.LoadSceneAsync(mapConfiguration.mapName, LoadSceneMode.Additive);
        }

        /// <summary>
        /// Configure the scene
        /// </summary>
        public void ConfigureScene(Transform rootTransform = null)
        {
        }

        /// <summary>
        /// Set up the UI for map configuration
        /// </summary>
        public void LoadUI()
        {
            mapUISelecor.options.Clear();
            for (int i = 0; i < SceneManager.sceneCountInBuildSettings; ++i)
            {
                bool sceneNameValid = true;
                var sceneName = SceneUtility.GetScenePathByBuildIndex(i).Split('/').Last().Replace(".unity", "");
                foreach (var scene in forbiddenSceneNames)
                {
                    if (scene == sceneName)
                    {
                        sceneNameValid = false;
                        break;
                    }
                }
                if (sceneNameValid)
                {
                    mapUISelecor.options.Add(
                        new Dropdown.OptionData(
                            sceneName
                        )
                    );
                }
            }
            mapUISelecor.value = 0;
            mapUISelecor.RefreshShownValue();
        }

        /// <summary>
        /// Load and validate config.
        /// </summary>
        public bool LoadConfig(AWSIMConfiguration config)
        {
            this.mapConfiguration = config.mapConfiguration;

            // Validate config
            if (SceneUtility.GetBuildIndexByScenePath(mapConfiguration.mapName) < 0)
            {
                Log(LogLevel.LOG_ERROR, $"Map '{mapConfiguration.mapName}' not found.");
                return false;
            }

            foreach (var scene in forbiddenSceneNames)
            {
                if (scene == mapConfiguration.mapName)
                {
                    Log(LogLevel.LOG_ERROR, $"Scene name '{mapConfiguration.mapName}' is reserved and cannot be used.");
                    return false;
                }
            }

            return true;
        }
    }

}
