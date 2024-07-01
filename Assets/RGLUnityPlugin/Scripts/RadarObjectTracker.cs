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

using System;
using UnityEngine;

namespace RGLUnityPlugin
{
    [Serializable]
    [RequireComponent(typeof(RadarSensor))]
    public class RadarObjectTracker : MonoBehaviour
    {
        [Tooltip("The maximum distance between a radar detection and other closest detection classified as the same tracked object (in meters)")]
        [Min(0.0f)] public float objectDistanceThreshold = 1.0f;

        [Tooltip("The maximum azimuth difference between a radar detection and other closest detection classified as the same tracked object (in degrees)")]
        [Min(0.0f)] public float objectAzimuthThreshold = 10.0f;

        [Tooltip("The maximum elevation difference between a radar detection and other closest detection classified as the same tracked object (in degrees)")]
        [Min(0.0f)] public float objectElevationThreshold = 5.0f;

        [Tooltip("The maximum radial speed difference between a radar detection and other closest detection classified as the same tracked object (in meters per second)")]
        [Min(0.0f)] public float objectRadialSpeedThreshold = 1.5f;

        [Tooltip("The maximum distance between predicted (based on previous frame) and newly detected object position to match objects between frames (in meters)")]
        [Min(0.0f)] public float maxMatchingDistance = 1.0f;

        [Tooltip("The maximum time that object state can be predicted until it will be declared lost unless measured again (in milliseconds)")]
        [Min(0.0f)] public float maxPredictionTimeFrame = 300.0f;

        [Tooltip("The maximum object's velocity to be qualified as stationary (in meters per seconds)")]
        [Min(0.0f)] public float movementSensitivity = 0.01f;

        private RadarSensor radarSensor;
        private RGLNodeSequence rglSubgraphRadarTrackObjects;
        private const string RadarTrackObjectsNodeId = "RADAR_TRACKING";

        public void Awake()
        {
            if (!IsRadarObjectTrackingAvailable())
            {
                Debug.LogError("Loaded RGL plugin does not include support for Radar Object Tracking, removing component");
                Destroy(this);
                return;
            }

            rglSubgraphRadarTrackObjects = new RGLNodeSequence()
                .AddNodePointsRadarTrackObjects(RadarTrackObjectsNodeId, objectDistanceThreshold,
                    objectAzimuthThreshold * Mathf.Deg2Rad, objectElevationThreshold * Mathf.Deg2Rad,
                    objectRadialSpeedThreshold, maxMatchingDistance, maxPredictionTimeFrame, movementSensitivity);
        }

        public void Start()
        {
            radarSensor = GetComponent<RadarSensor>();
            radarSensor.ConnectToWorldFrame(rglSubgraphRadarTrackObjects);

            if (RadarObjectClassProvider.Instance != null)
            {
                RadarObjectClassProvider.Instance.OnNewConfig += OnRadarObjectClassesChanged;
                OnRadarObjectClassesChanged();
            }
        }

        public void OnEnable()
        {
            rglSubgraphRadarTrackObjects?.SetActive(RadarTrackObjectsNodeId, true);
        }

        public void OnDisable()
        {
            rglSubgraphRadarTrackObjects?.SetActive(RadarTrackObjectsNodeId, false);
        }

        public void OnDestroy()
        {
            rglSubgraphRadarTrackObjects?.Clear();
            rglSubgraphRadarTrackObjects = null;
        }

        public void OnValidate()
        {
            if (rglSubgraphRadarTrackObjects == null)
            {
                return;
            }

            rglSubgraphRadarTrackObjects.UpdateNodePointsRadarTrackObjects(RadarTrackObjectsNodeId, objectDistanceThreshold,
                objectAzimuthThreshold * Mathf.Deg2Rad, objectElevationThreshold * Mathf.Deg2Rad,
                objectRadialSpeedThreshold, maxMatchingDistance, maxPredictionTimeFrame, movementSensitivity);
        }

        public void OnRadarObjectClassesChanged()
        {
            if (RadarObjectClassProvider.Instance != null && RadarObjectClassProvider.Instance.enabled)
            {
                var mapping = RadarObjectClassProvider.Instance.GetEntityIdToRadarClassMapping();
                rglSubgraphRadarTrackObjects.SetNodeRadarClasses(RadarTrackObjectsNodeId, mapping.Item1, mapping.Item2);
            }
            else
            {
                rglSubgraphRadarTrackObjects.SetNodeRadarClasses(RadarTrackObjectsNodeId, Array.Empty<int>(),
                    Array.Empty<RGLRadarObjectClass>());
            }
        }

        public void Connect(RGLNodeSequence nodeSequence)
        {
            RGLNodeSequence.Connect(rglSubgraphRadarTrackObjects, nodeSequence);
        }

        public static bool IsRadarObjectTrackingAvailable()
        {
            return RGLNativeAPI.HasExtension(RGLExtension.RGL_EXTENSION_UDP);
        }
    }
}
