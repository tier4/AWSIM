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
using Awsim.Common;
using System;

namespace Awsim.Usecase.PcdGeneration
{
    public class PcdGenerator : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Imported OSM file. Mapping is conducted along all centerlines of lanelets in the OSM.")]
        OsmDataContainer _osmDataContainer;

        [SerializeField]
        [Tooltip("Game object containing sensors to capture pointcloud. It will be warped along centerlines of lanelets.")]
        Transform _vehicleTransform;

        [SerializeField]
        RGLMappingAdapter _rglMappingAdapter;

        [SerializeField]
        [Tooltip("Result PCD file name. On Editor/Windows, it will be saved in Assets/")]
        string _outputPcdFilePath = "output.pcd";

        [SerializeField]
        [Tooltip("Distance in meters between consecutive warps along the centerline of a lanelet.")]
        float _captureLocationInterval = 6f;

        [SerializeField]
        [Tooltip("World origin in ROS coordinate systems, will be added to every point coordinates")]
        Vector3 _worldOriginROS;

        // [SerializeField]
        // [Tooltip("Configurable visualization of the loaded lanelet map")]
        // LaneletVisualizer _laneletVisualizer;
        Queue<Pose> _capturePoseQueue;

        bool _saved = false;
        public Action OnPcdSaved;

        public void Initialize()
        {
            if (_rglMappingAdapter == null)
            {
                Debug.LogError($"Could not find mapping sensor in {_vehicleTransform.name}. Disabling PointCloudMapper!");
                enabled = false;
                return;
            }

            _rglMappingAdapter.Initialize(_worldOriginROS, $"{Application.dataPath}/{_outputPcdFilePath}");

            var laneletMap = new OsmToLaneletMap(_worldOriginROS).Convert(_osmDataContainer.Data);

            var start = Time.realtimeSinceStartup;
            _capturePoseQueue = new Queue<Pose>(LaneletMapToPoses(laneletMap, _captureLocationInterval));
            var computeTimeMs = (Time.realtimeSinceStartup - start) * 1000f;
            Debug.Log($"Will visit {_capturePoseQueue.Count} points; computed in {computeTimeMs} ms");

            // _laneletVisualizer.Initialize(laneletMap);
            // _laneletVisualizer.CreateCenterline(transform);
        }

        public void OnUpdate()
        {
            Debug.Log($"PointCloudMapper: {_capturePoseQueue.Count} captures left");
            if (_capturePoseQueue.Count == 0)
            {
                if (_saved)
                    return;

                SavePcd();
                enabled = false;
                _saved = true;
                OnPcdSaved?.Invoke();
                return;
            }

            var currentPose = _capturePoseQueue.Dequeue();
            _vehicleTransform.position = currentPose.position;
            _vehicleTransform.transform.rotation = currentPose.rotation;

            _rglMappingAdapter.Capture();
        }

        void OnDestroy()
        {
            if (enabled)
            {
                SavePcd();
            }
            _rglMappingAdapter.Dispose();
        }

        void SavePcd()
        {
            Debug.Log($"Writing PCD to {Application.dataPath}/{_outputPcdFilePath}");
            _rglMappingAdapter.SavePcd();
            Debug.Log("PCL data saved successfully");
        }

        static IEnumerable<Pose> LaneletMapToPoses(LaneletMap laneletMap, float jumpDistance)
        {
            foreach (var laneletData in laneletMap.Lanelets.Values)
            {
                float distanceVisited = 0.0f;
                Vector3[] centerPoints = laneletData.CalculateCenterline();
                BezierPath bezierPath = new BezierPathFactory().CreateBezierPath(centerPoints);

                while (distanceVisited <= bezierPath.Length)
                {
                    yield return bezierPath.TangentPose(distanceVisited);
                    distanceVisited += jumpDistance;
                }
            }
        }
    }
}
