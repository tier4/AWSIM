using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    [RequireComponent(typeof(V2I))]
    public class V2IRos2Publisher : MonoBehaviour
    {
        [SerializeField, Tooltip("On this topic, the traffic_signals are published (as a ")]
        string trafficSignalsTopic = "/v2x/traffic_signals";

        public string frameId = "v2i";
        
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };
        
        IPublisher<autoware_perception_msgs.msg.TrafficSignalArray> trafficSignalsPublisher;
        autoware_perception_msgs.msg.TrafficSignalArray trafficSignalArrayMsg;

        V2I v2iComponent;

        void Start()
        {
            v2iComponent = GetComponent<V2I>();
            v2iComponent.OnOutputData += UpdateMessageAndPublish;

            trafficSignalArrayMsg = new autoware_perception_msgs.msg.TrafficSignalArray();

            var qos = qosSettings.GetQoSProfile();
            trafficSignalsPublisher = SimulatorROS2Node.CreatePublisher<autoware_perception_msgs.msg.TrafficSignalArray>(trafficSignalsTopic, qos);
        }

        void UpdateMessageAndPublish(V2I.OutputData outputData)
        {
            UpdateTrafficSignalArrayMsg(outputData);
            trafficSignalsPublisher.Publish(trafficSignalArrayMsg);
        }

        private void UpdateTrafficSignalArrayMsg(V2I.OutputData data)
        {
            var ts_list = new List<autoware_perception_msgs.msg.TrafficSignal>();
            foreach (var trafficLight in data.trafficLights)
            {
                var ts = new autoware_perception_msgs.msg.TrafficSignal();
                ts.Traffic_signal_id = trafficLight.LaneletElementID;
                //Get bulbData
                var bulbData = trafficLight.GetBulbData();
                //Adjust container size
                ts.Elements = new autoware_perception_msgs.msg.TrafficLightElement[bulbData.Length];
                //Fill TrafficSignal with bulbData
                for (int i = 0; i < bulbData.Length; ++i)
                {
                    ts.Elements[i] = new autoware_perception_msgs.msg.TrafficLightElement();
                    ts.Elements[i].Color = TrafficLightROS2Utility.UnityToRosBulbColor(bulbData[i].Color);
                    ts.Elements[i].Shape = TrafficLightROS2Utility.UnityToRosBulbShape(bulbData[i].Type);
                    ts.Elements[i].Status = TrafficLightROS2Utility.UnityToRosBulbStatus(bulbData[i].Status);
                    ts.Elements[i].Confidence = 1.0f;
                }
                //Add TrafficLight signal to list
                ts_list.Add(ts);
            }
            trafficSignalArrayMsg.Stamp = SimulatorROS2Node.GetCurrentRosTime();
            trafficSignalArrayMsg.Signals = ts_list.ToArray();
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<autoware_perception_msgs.msg.TrafficSignalArray>(trafficSignalsPublisher);
        }
    }

}
