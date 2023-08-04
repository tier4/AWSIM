using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    [RequireComponent(typeof(V2I))]
    public class V2IRos2Publisher : MonoBehaviour
    {
        public enum TrafficSignalID{
            RelationID,
            WayID
        }

        public TrafficSignalID trafficSignalID;

        [SerializeField, Tooltip("On this topic, the traffic_signals are published (as a ")]
        string trafficSignalsTopic = "/v2x/traffic_signals";
        
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
            var trafficSignalList = new List<autoware_perception_msgs.msg.TrafficSignal>();
            var allRelationID = new List<long>();
            foreach (var trafficLight in data.trafficLights)
            {
                var trafficLightLaneletID = trafficLight.GetComponentInParent<TrafficLightLaneletID>();
                if (trafficLightLaneletID != null)
                {
                    var ids = new List<long>();
                    if (trafficSignalID == TrafficSignalID.RelationID)
                    {
                        ids = trafficLightLaneletID.relationID;
                    }
                    else if (trafficSignalID == TrafficSignalID.WayID)
                    {
                        ids.Add(trafficLightLaneletID.wayID);
                    }
                    foreach (var relationID in ids)
                    {
                        var trafficSignalMsg = new autoware_perception_msgs.msg.TrafficSignal();
                        if (allRelationID.Contains(relationID))
                        {
                            continue;
                        }
                        trafficSignalMsg.Traffic_signal_id = relationID;
                        //Get bulbData
                        var trafficLightBulbData = trafficLight.GetBulbData();
                        //Fill TrafficSignal with bulbData
                        var trafficLightElementList = new List<autoware_perception_msgs.msg.TrafficLightElement>();
                        foreach (var bulbData in trafficLightBulbData)
                        {
                            if (isBulbTurnOn(bulbData.Status))
                            {
                                var trafficLightElementMsg = new autoware_perception_msgs.msg.TrafficLightElement();
                                trafficLightElementMsg.Color = V2IROS2Utility.UnityToRosBulbColor(bulbData.Color);
                                trafficLightElementMsg.Shape = V2IROS2Utility.UnityToRosBulbShape(bulbData.Type);
                                trafficLightElementMsg.Status = V2IROS2Utility.UnityToRosBulbStatus(bulbData.Status);
                                trafficLightElementMsg.Confidence = 1.0f;
                                trafficLightElementList.Add(trafficLightElementMsg);
                            }
                        }
                        //Add TrafficLight signal to list
                        trafficSignalMsg.Elements = trafficLightElementList.ToArray();
                        trafficSignalList.Add(trafficSignalMsg);
                        allRelationID.Add(relationID);
                    }
                }
            }
            trafficSignalArrayMsg.Stamp = SimulatorROS2Node.GetCurrentRosTime();
            trafficSignalArrayMsg.Signals = trafficSignalList.ToArray();
        }

        private bool isBulbTurnOn(TrafficLight.BulbStatus bulbStatus)
        {
            return bulbStatus == TrafficLight.BulbStatus.SOLID_ON || bulbStatus == TrafficLight.BulbStatus.FLASHING;
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<autoware_perception_msgs.msg.TrafficSignalArray>(trafficSignalsPublisher);
        }
    }

}
