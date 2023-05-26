// Copyright 2023 Tier4.
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

using UnityEditor;
using UnityEngine;

namespace AWSIM.Lanelet
{
    public class LaneletBoundsVisualizerWindow : EditorWindow
    {
        [SerializeField] private OsmDataContainer osm;
        [SerializeField] private LaneletBoundsVisualizer.WaypointSettings waypointSettings = LaneletBoundsVisualizer.WaypointSettings.Default();
        private SerializedObject serializedObject;

        [MenuItem("AWSIM/Visualize/Load Lanelet Bounds")]
        private static void ShowWindow()
        {
            var window = GetWindow(typeof(LaneletBoundsVisualizerWindow), true, "LaneletBoundsVisualizer");
            window.Show();
        }

        private void OnEnable()
        {
            serializedObject = new SerializedObject(this);
        }

        private void OnGUI()
        {
            serializedObject.Update();

            // Input fields
            var osmProperty = serializedObject.FindProperty("osm");
            EditorGUILayout.PropertyField(osmProperty, true);
            var settingsProperty = serializedObject.FindProperty("waypointSettings");
            settingsProperty.isExpanded = true;
            EditorGUILayout.PropertyField(settingsProperty, true);

            serializedObject.ApplyModifiedProperties();

            if (GUILayout.Button("Load with Waypoint Setting"))
            {
                var referencePoint = Environment.Instance.MgrsOffsetPosition;
                var loader = new LaneletBoundsVisualizer();
                loader.SetWaypointSettings(waypointSettings);

                loader.Load(osm.Data, referencePoint, Environment.Instance.gameObject, false);
            }
            else if (GUILayout.Button("Load Raw Data"))
            {
                var referencePoint = Environment.Instance.MgrsOffsetPosition;
                var loader = new LaneletBoundsVisualizer();
                loader.SetWaypointSettings(waypointSettings);

                loader.Load(osm.Data, referencePoint, Environment.Instance.gameObject, true);
            }
        }
    }
}
