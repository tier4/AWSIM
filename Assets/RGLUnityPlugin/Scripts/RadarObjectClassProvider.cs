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
using System;
using System.Collections.Generic;

namespace RGLUnityPlugin
{
    [Serializable]
    public struct RadarObjectClass
    {
        [field: SerializeField]
        [field: Tooltip("Radar object class for given entity ids range")]
        public RGLRadarObjectClass ClassId  { get; private set; }

        [field: SerializeField]
        [field: Tooltip("The beginning of entity id range")]
        public int EntityIdRangeBegin  { get; private set; }

        [field: SerializeField]
        [field: Tooltip("The end of entity id range")]
        public int EntityIdRangeEnd  { get; private set; }
    }

    /// <summary>
    /// The singleton component to configure the radar object classes for the radar sensor.
    /// Note: This is a private feature. It requires replacing the RGL binary with the RGL private build binary.
    /// </summary>
    public class RadarObjectClassProvider : MonoBehaviour
    {
        // Singleton pattern
        public static RadarObjectClassProvider Instance { get; private set; }

        // Delegate to notify that the radar object classes have been changed
        public delegate void OnNewConfigDelegate();
        public OnNewConfigDelegate OnNewConfig;

        [field: SerializeField]
        [field: Tooltip("List of entity ids to radar object class mapping")]
        public List<RadarObjectClass> EntityIdToRadarClassMapping { get; private set; }

        private void Awake()
        {
            if (!RadarObjectTracker.IsRadarObjectTrackingAvailable())
            {
                Debug.LogError("Loaded RGL plugin does not include support for Radar Object Tracking, removing component");
                Destroy(this);
                return;
            }

            if (Instance != null && Instance != this)
            {
                Debug.LogError("RadarObjectClassProvider is already on the scene. Removing this component");
                Destroy(this);
                return;
            }
            Instance = this;
        }

        public Tuple<int[], RGLRadarObjectClass[]> GetEntityIdToRadarClassMapping()
        {
            var entityIds = new List<int>();
            var radarObjectClasses = new List<RGLRadarObjectClass>();
            foreach (var mappingElement in EntityIdToRadarClassMapping)
            {
                if (!(mappingElement.EntityIdRangeBegin <= mappingElement.EntityIdRangeEnd))
                {
                    throw new ArgumentOutOfRangeException(nameof(mappingElement.EntityIdRangeBegin),
                        "The beginning of entity id range must be lower or equal to end of that range");
                }

                for (var i = mappingElement.EntityIdRangeBegin; i <= mappingElement.EntityIdRangeEnd; ++i)
                {
                    entityIds.Add(i);
                    radarObjectClasses.Add(mappingElement.ClassId);
                }
            }

            return new Tuple<int[], RGLRadarObjectClass[]>(entityIds.ToArray(), radarObjectClasses.ToArray());
        }

        private void OnValidate()
        {
            OnNewConfig?.Invoke();
        }

        private void OnEnable()
        {
            OnNewConfig?.Invoke();
        }

        private void OnDisable()
        {
            OnNewConfig?.Invoke();
        }
    }
}
