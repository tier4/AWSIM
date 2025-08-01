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

namespace Awsim.Usecase.TrafficSimulation
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(StopLine))]
    public class StopLineEditor : Editor
    {
        [DrawGizmo(GizmoType.InSelectionHierarchy | GizmoType.Pickable)]
        static void DrawGizmo(StopLine stopLine, GizmoType gizmoType)
        {
            var matCache = Gizmos.matrix;
            var colorCache = Gizmos.color;

            var direction = stopLine.Points[1] - stopLine.Points[0];
            var center = (stopLine.Points[0] + stopLine.Points[1]) / 2;
            var rotation = Quaternion.LookRotation(direction);
            Gizmos.matrix = Matrix4x4.TRS(center, rotation, Vector3.one);
            Gizmos.color = stopLine.HasStopSign ? Color.red : Color.white;
            var size = new Vector3(0.3f, 1f, direction.magnitude);
            Gizmos.DrawCube(Vector3.zero, size);

            Gizmos.matrix = matCache;
            Gizmos.color = colorCache;
        }

        void OnSceneGUI()
        {
            // var stopLine = target as StopLine;
            // TODO: Handle implementation
        }
    }
}