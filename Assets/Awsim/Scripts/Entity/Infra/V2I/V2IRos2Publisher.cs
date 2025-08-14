// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System.Collections.Generic;
using UnityEngine;
using ROS2;
using Awsim.Common;

namespace Awsim.Entity
{
    [RequireComponent(typeof(V2I))]
    public class V2IRos2Publisher : MonoBehaviour
    {
        public enum TrafficSignalID
        {
            RelationID,
            WayID
        }

        public TrafficSignalID trafficSignalID;

        [SerializeField, Tooltip("On this topic, the traffic_signals are published (as a ")]
        string trafficSignalsTopic = "/v2x/traffic_signals";

        public QosSettings qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                                         DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                                         HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                                         1);

        IPublisher<autoware_perception_msgs.msg.TrafficLightGroupArray> trafficLightGroupPublisher;
        autoware_perception_msgs.msg.TrafficLightGroupArray trafficLightGroupMsg;

        V2I v2iComponent;

        public void Initialize()
        {
            v2iComponent = GetComponent<V2I>();
            v2iComponent.OnOutput += UpdateMessageAndPublish;

            trafficLightGroupMsg = new autoware_perception_msgs.msg.TrafficLightGroupArray();

            var qos = qosSettings.GetQosProfile();
            trafficLightGroupPublisher = AwsimRos2Node.CreatePublisher<autoware_perception_msgs.msg.TrafficLightGroupArray>(trafficSignalsTopic, qos);
        }

        void UpdateMessageAndPublish(V2I.OutputData outputData)
        {
            UpdateTrafficLightGroupMsg(outputData);
            trafficLightGroupPublisher.Publish(trafficLightGroupMsg);
        }

        void UpdateTrafficLightGroupMsg(V2I.OutputData data)
        {
            var trafficLightGroup = new List<autoware_perception_msgs.msg.TrafficLightGroup>();
            var allRelationId = new List<long>();
            foreach (var trafficLight in data.trafficLights)
            {
                var trafficLightLaneletId = trafficLight.LaneletId;

                var ids = new List<long>();
                if (trafficSignalID == TrafficSignalID.RelationID)
                {
                    ids = trafficLightLaneletId.relationId;
                }
                else if (trafficSignalID == TrafficSignalID.WayID)
                {
                    ids.Add(trafficLightLaneletId.wayId);
                }
                foreach (var relationID in ids)
                {
                    var trafficLightGroupMsg = new autoware_perception_msgs.msg.TrafficLightGroup();
                    if (allRelationId.Contains(relationID))
                    {
                        continue;
                    }
                    trafficLightGroupMsg.Traffic_light_group_id = relationID;
                    //Get bulbData
                    var trafficLightBulbData = trafficLight.GetBulbData();
                    //Fill TrafficSignal with bulbData
                    var trafficLightElementList = new List<autoware_perception_msgs.msg.TrafficLightElement>();
                    foreach (var bulbData in trafficLightBulbData)
                    {
                        if (IsBulbTurnOn(bulbData.Status))
                        {
                            var trafficLightElementMsg = new autoware_perception_msgs.msg.TrafficLightElement();
                            trafficLightElementMsg.Color = V2IRos2Converter.UnityToRosBulbColor(bulbData.Color);
                            trafficLightElementMsg.Shape = V2IRos2Converter.UnityToRosBulbShape(bulbData.Type);
                            trafficLightElementMsg.Status = V2IRos2Converter.UnityToRosBulbStatus(bulbData.Status);
                            trafficLightElementMsg.Confidence = 1.0f;
                            trafficLightElementList.Add(trafficLightElementMsg);
                        }
                    }
                    //Add TrafficLight signal to list
                    trafficLightGroupMsg.Elements = trafficLightElementList.ToArray();
                    trafficLightGroup.Add(trafficLightGroupMsg);
                    allRelationId.Add(relationID);
                }
            }
            AwsimRos2Node.UpdateROSClockTime(trafficLightGroupMsg.Stamp);
            trafficLightGroupMsg.Traffic_light_groups = trafficLightGroup.ToArray();
        }

        bool IsBulbTurnOn(TrafficLight.BulbStatus bulbStatus)
        {
            return bulbStatus == TrafficLight.BulbStatus.SolidOn || bulbStatus == TrafficLight.BulbStatus.Frashing;
        }

        void OnDestroy()
        {
            AwsimRos2Node.RemovePublisher<autoware_perception_msgs.msg.TrafficLightGroupArray>(trafficLightGroupPublisher);
        }
    }

}