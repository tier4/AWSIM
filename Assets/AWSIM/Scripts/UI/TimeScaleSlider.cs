using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;

namespace AWSIM
{
    [System.Serializable]
    public class SimulationSettings
    {
        public float timeScale;
    }

    public class TimeScaleSlider : MonoBehaviour
    {
        [SerializeField] Text timeScaleText;
        [SerializeField] Slider timeScaleSlider;

        private TextAsset jsonFile;
        private string filePath;

        private void Start()
        {
            filePath = Application.dataPath + "/StreamingAssets/simulationSettings.json";
            if (File.Exists(filePath))
            {
                string dataAsJson = File.ReadAllText(filePath);
                SimulationSettings simSettings = JsonUtility.FromJson<SimulationSettings>(dataAsJson);
                if (simSettings.timeScale < 0.0f)
                {
                    Debug.LogError("Time scale cannot be negative! Setting to 0.");
                    simSettings.timeScale = 0.0f;
                }
                simSettings.timeScale = Math.Min(simSettings.timeScale, 1.0f);
                timeScaleSlider.value = simSettings.timeScale;
                Debug.Log($"Time scale set to: {simSettings.timeScale}");
            }
            Time.timeScale = timeScaleSlider.value;
            timeScaleText.text = "x " + timeScaleSlider.value.ToString("F2");
        }

        public void SetTimeScale(float timeScale)
        {
            Time.timeScale = timeScale;
            timeScaleText.text = "x " + timeScale.ToString("F2"); 
        }
    }
}