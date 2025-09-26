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

using UnityEditor;
using UnityEngine;

namespace Awsim.Common
{
    public class LaneletLoaderWindow : EditorWindow
    {
        [SerializeField] OsmDataContainer _osm;
        [SerializeField] LaneletLoader.WaypointSettings _waypointSettings = LaneletLoader.WaypointSettings.Default();
        [SerializeField] GameObject _rootObject;
        SerializedObject _serializedObject;

        [MenuItem("AWSIM/Common/Load Lanelet")]
        static void ShowWindow()
        {
            var window = GetWindow(typeof(LaneletLoaderWindow), true, "LaneletLoader");
            window.Show();
        }

        void OnEnable()
        {
            _serializedObject = new SerializedObject(this);

        }

        void OnGUI()
        {
            _serializedObject.Update();

            // Input fields
            var osmProperty = _serializedObject.FindProperty("_osm");
            EditorGUILayout.PropertyField(osmProperty, true);
            var settingsProperty = _serializedObject.FindProperty("_waypointSettings");
            settingsProperty.isExpanded = true;
            EditorGUILayout.PropertyField(settingsProperty, true);
            var rootObject = _serializedObject.FindProperty("_rootObject");
            EditorGUILayout.PropertyField(rootObject, true);

            _serializedObject.ApplyModifiedProperties();

            if (GUILayout.Button("Load"))
            {
                var referencePoint = MgrsPosition.Instance.Mgrs.Position;
                var loader = new LaneletLoader();
                loader.SetWaypointSettings(_waypointSettings);
                loader.Load(_osm.Data, referencePoint, _rootObject);        // TODO:
            }
        }
    }
}
