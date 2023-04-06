using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    using TrafficSignalArray = autoware_perception_msgs.msg.TrafficSignalArray;
    using TrafficSignal = autoware_perception_msgs.msg.TrafficSignal;
    using TrafficLightElement = autoware_perception_msgs.msg.TrafficLightElement;

    public class TrafficSignals : MonoBehaviour
    {
        private float _timer = 0;
        private bool initialized = false;
        //TODO: ensure that this list contains the objects TrafficLights whose data we want to publish (may be updated in the background)
        [SerializeField] private List<TrafficLight> nearbyTrafficLights;
        private IPublisher<TrafficSignalArray> signalsPublisher;
        private TrafficSignalArray signalArray;

        [Header("ROS Communication Settings")]
        //QoS settings for communication with ROS"
        [SerializeField, Tooltip("QoS settings for communication with ROS")] QoSSettings qosSettings;
        [Header("Output")]
        //The topic for publication 'is stuck' state
        [SerializeField, Tooltip("On this topic, the traffic_signals are published (as a ")] string signalsTopic = "/v2x/traffic_signals";
        [SerializeField, Tooltip("traffic_signals publication frequency - number of publications per second"), Range(1.0f, 100.0f)] int publishFrequency = 30;

        void Start()
        {
            qosSettings = new QoSSettings();
            signalArray = new TrafficSignalArray();
            signalsPublisher = SimulatorROS2Node.CreatePublisher<TrafficSignalArray>(signalsTopic, qosSettings.GetQoSProfile());
            initialized = true;
        }
        void TMP_UpdateTrafficLights()
        {
            foreach (var trafficLight in nearbyTrafficLights)
            {
                trafficLight.LaneletElementID = 1111;
                var bulbData1 = new TrafficLight.BulbData[]
                {
                    new TrafficLight.BulbData(TrafficLight.BulbType.GREEN_BULB, TrafficLight.BulbColor.GREEN, TrafficLight.BulbStatus.SOLID_OFF),
                    new TrafficLight.BulbData(TrafficLight.BulbType.YELLOW_BULB, TrafficLight.BulbColor.YELLOW, TrafficLight.BulbStatus.SOLID_ON)
                };
                trafficLight.SetBulbData(bulbData1);
            }
        }

        bool NeedToPublish()
        {
            _timer += Time.deltaTime;
            var interval = 1.0f / publishFrequency;
            interval -= 0.00001f;
            if (_timer < interval)
                return false;
            _timer = 0;
            return true;
        }

        void UpdateSignalArray()
        {
            var ts_list = new List<TrafficSignal>();
            foreach (var trafficLight in nearbyTrafficLights)
            {
                var ts = new TrafficSignal();
                ts.Traffic_signal_id = trafficLight.LaneletElementID;
                //Get bulbData
                var bulbData = trafficLight.GetBulbData();
                //Adjust container size
                ts.Elements = new TrafficLightElement[bulbData.Length];
                //Fill TrafficSignal with bulbData
                for (int i = 0; i < bulbData.Length; ++i)
                {
                    ts.Elements[i] = new TrafficLightElement();
                    ts.Elements[i].Color = TrafficLightROS2Utility.UnityToRosBulbColor(bulbData[i].Color);
                    ts.Elements[i].Shape = TrafficLightROS2Utility.UnityToRosBulbShape(bulbData[i].Type);
                    ts.Elements[i].Status = TrafficLightROS2Utility.UnityToRosBulbStatus(bulbData[i].Status);
                    ts.Elements[i].Confidence = 1.0f;
                }
                //Add TrafficLight signal to list
                ts_list.Add(ts);
            }
            signalArray.Stamp = SimulatorROS2Node.GetCurrentRosTime();
            signalArray.Signals = ts_list.ToArray();
        }

        void FixedUpdate()
        {
            if (!initialized)
                return;

            TMP_UpdateTrafficLights(); //TrafficIntesection is responsible for this update
            if (NeedToPublish())
            {
                UpdateSignalArray();
                signalsPublisher.Publish(signalArray);
            }
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<TrafficSignalArray>(signalsPublisher);
        }

    }
}
