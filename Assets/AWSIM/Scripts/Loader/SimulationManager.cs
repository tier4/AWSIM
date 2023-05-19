using UnityEngine;
using System;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

namespace AWSIM.Loader 
{
    public class SimulationManager : MonoBehaviour, IConfigurableManager
    {
        public string simulationSceneName = "AWSIMSimulation";
        [HideInInspector]
        public SimulationConfiguration simulationConfiguration;
        public Toggle mapTrafficToggle;
        public Action<LogLevel, string> Log { get; set; }

        public AsyncOperation ResetLoader()
        {
            return SceneManager.LoadSceneAsync(SceneManager.GetActiveScene().name, LoadSceneMode.Single);
        }

        public AsyncOperation LoadSimulation()
        {
            Log(LogLevel.LOG_INFO, $"Loading simulation");
            return SceneManager.LoadSceneAsync(simulationSceneName, LoadSceneMode.Additive);
        }

        public void ConfigureScene(Transform rootTransform = null)
        {
            // Set traffic on/off
            var trafficSims = GameObject.FindObjectsOfType<TrafficSimulation.TrafficManager>();
            foreach (var trafficSim in trafficSims)
            {
                trafficSim.gameObject.SetActive(simulationConfiguration.useTraffic);
            }

            // Set scene time scale
            DemoUI demoUi = GameObject.FindObjectOfType<DemoUI>();
            demoUi.SetTimeScale(simulationConfiguration.timeScale);
            demoUi.TimeScaleSlider.value = simulationConfiguration.timeScale;
        }

        public void LoadUI()
        {
        }

        public bool LoadConfig(AWSIMConfiguration config)
        {
            this.simulationConfiguration = config.simulationConfiguration;

            // Validate config
            if (simulationConfiguration.timeScale < 0.0f || simulationConfiguration.timeScale > 1.0f)
            {
                Log(LogLevel.LOG_ERROR, $"Invalid time scale value. Must be higher or equal 0 and lower of equal 1.0.");
                return false;
            }

            return true;
        }
    }

}
