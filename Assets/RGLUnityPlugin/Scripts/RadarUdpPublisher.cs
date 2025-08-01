// Copyright 2024 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;

namespace RGLUnityPlugin
{
    [RequireComponent(typeof(RadarObjectTracker))]
    public class RadarUdpPublisher : MonoBehaviour
    {
        public string sourceIP = "0.0.0.0";
        public string destinationIP = "255.255.255.255";
        public int destinationPort = 42102;

        private string sourceIPOnAwake;
        private string destinationIPOnAwake;
        private int destinationPortOnAwake;

        private RGLNodeSequence rglSubgraphUdpPublishing;

        private const string udpPublishingNodeId = "UDP_PUBLISHING";

        private RadarObjectTracker radarObjectTracker;

        private void Awake()
        {
            if (!IsRadarUdpPublishingAvailable())
            {
                Debug.LogError("Loaded RGL plugin does not include support for Radar Udp Publishing, removing component");
                Destroy(this);
                return;
            }

            sourceIPOnAwake = sourceIP;
            destinationIPOnAwake = destinationIP;
            destinationPortOnAwake = destinationPort;

            if (!NetworkUtilities.IsValidIpAddress(sourceIPOnAwake))
            {
                var detailedName = transform.parent != null ? $"{transform.parent.name}:{name}" : name;
                Debug.LogError($"{detailedName}: IP address '{sourceIPOnAwake}' is invalid. " +
                               "Disabling component. Please restart the simulation with a correct IP address.");
                OnDisable();
                return;
            }

            rglSubgraphUdpPublishing = new RGLNodeSequence()
                .AddNodePublishUdpObjectList(udpPublishingNodeId, sourceIPOnAwake, destinationIPOnAwake, destinationPortOnAwake);
        }

        private void Start()
        {
            radarObjectTracker = GetComponent<RadarObjectTracker>();
            radarObjectTracker.Connect(rglSubgraphUdpPublishing);
        }

        public void OnValidate()
        {
            if (rglSubgraphUdpPublishing == null)
            {
                return;
            }

            if (sourceIPOnAwake != sourceIP)
            {
                sourceIP = sourceIPOnAwake;
                Debug.LogWarning("`sourceIP` parameter cannot be changed in simulation runtime");
            }

            if (destinationIPOnAwake != destinationIP)
            {
                destinationIP = destinationIPOnAwake;
                Debug.LogWarning("`destinationIP` parameter cannot be changed in simulation runtime");
            }

            if (destinationPortOnAwake != destinationPort)
            {
                destinationPort = destinationPortOnAwake;
                Debug.LogWarning("`destinationPort` parameter cannot be changed in simulation runtime");
            }
        }

        public void OnEnable()
        {
            rglSubgraphUdpPublishing?.SetActive(udpPublishingNodeId, true);
        }

        public void OnDisable()
        {
            rglSubgraphUdpPublishing?.SetActive(udpPublishingNodeId, false);
            enabled = false;
        }

        public void OnDestroy()
        {
            rglSubgraphUdpPublishing?.Clear();
            rglSubgraphUdpPublishing = null;
        }

        private bool IsRadarUdpPublishingAvailable()
        {
            return RGLNativeAPI.HasExtension(RGLExtension.RGL_EXTENSION_UDP);
        }
    }
}
