using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;

namespace AWSIM
{
    public class V2I : MonoBehaviour
    {
        [SerializeField,
        Tooltip("traffic_signals publication frequency - number of publications per second"),
        Range(1.0f, 100.0f)]
        int outputHz = 10;

        [SerializeField] Transform egoVehicleTransform;

        private TrafficLight[] allTrafficLights;

        public double egoDistanceToTrafficSignals = 150.0;
        
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
            allTrafficLights = GameObject.FindObjectsOfType<TrafficLight>();
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

            outputData.trafficLights = FindClosestTrafficLights(allTrafficLights, egoVehicleTransform.position,
                egoDistanceToTrafficSignals);

            OnOutputData.Invoke(outputData);
        }

        private TrafficLight[] FindClosestTrafficLights(TrafficLight[] trafficLights, Vector3 position, double radius = 10.0)
        {
            return trafficLights
                .Where(trafficLight =>
                {
                    var distance2D = GeometryUtility.Distance2D(trafficLight.transform.position, position);
                    return distance2D <= radius && trafficLight.LaneletElementID != 0;
                }).ToList().ToArray();
        }
    }


}
