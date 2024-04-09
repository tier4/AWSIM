using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using AWSIM.TrafficSimulation;

namespace AWSIM
{
    public class TrafficSettingsUI : MonoBehaviour
    {
        [SerializeField] TrafficManager trafficManager;
        [SerializeField] InputField seedInputField;
        [SerializeField] InputField maxVehicleCountInputField;
        [SerializeField] GameObject appliedTextObj;

        int seed = 0;
        int count = 0;

        void Start()
        {
            seed = trafficManager.seed;
            count = trafficManager.maxVehicleCount;

            seedInputField.text = seed.ToString();
            maxVehicleCountInputField.text = count.ToString();
        }

        public void RestartRandomTraffic()
        {
            if (seedInputField.text == string.Empty)
                seedInputField.text = seed.ToString();

            if (maxVehicleCountInputField.text == string.Empty)
                maxVehicleCountInputField.text = count.ToString();

            seed = Int32.Parse(seedInputField.text);
            count = Int32.Parse(maxVehicleCountInputField.text);

            trafficManager.Restart(seed, count);

            StartCoroutine(DisplayAppliedText());
        }

        IEnumerator DisplayAppliedText()
        {
            appliedTextObj.SetActive(true);
            yield return new WaitForSecondsRealtime(3f);
            appliedTextObj.SetActive(false);
        }
    }
}