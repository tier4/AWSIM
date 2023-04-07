using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    public class V2I : MonoBehaviour
    {
        [SerializeField,
        Tooltip("traffic_signals publication frequency - number of publications per second"),
        Range(1.0f, 100.0f)]
        int outputHz = 10;

        [SerializeField] 
        private GameObject trafficLightsParent;
        
        public class OutputData
        {
            public TrafficLight[] trafficLights;
        }

        public delegate void OnOutputDataDelegate(OutputData outputData);
        public OnOutputDataDelegate OnOutputData;

        OutputData outputData = new OutputData();
        float timer = 0;

        void Start()
        {
            outputData.trafficLights = trafficLightsParent.GetComponentsInChildren<TrafficLight>();
        }

        void FixedUpdate()
        {
            // Update timer.
            timer += Time.deltaTime;

            // Matching output to hz.
            var interval = 1.0f / (int)outputHz;
            interval -= 0.00001f;
            if (timer < interval)
                return;
            timer = 0;

            OnOutputData.Invoke(outputData);
        }
    }


}
