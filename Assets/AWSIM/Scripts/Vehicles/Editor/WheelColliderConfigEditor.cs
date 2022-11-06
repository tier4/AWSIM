using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace AWSIM
{
    [CustomEditor(typeof(WheelColliderConfig)), CanEditMultipleObjects]
    public class WheelColliderConfigEditor : Editor
    {
        WheelColliderConfig wheelColliderConfig;
        bool foldout = false;

        private void OnEnable()
        {
            wheelColliderConfig = target as WheelColliderConfig;

            // Cannnot edit WheelCollider value at inspector.
            wheelColliderConfig.WheelCollider.hideFlags = HideFlags.NotEditable;
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            var wc = wheelColliderConfig.WheelCollider;

            EditorGUI.BeginChangeCheck();

            // Radius.
            wc.radius = EditorGUILayout.FloatField("Radius (m)", wc.radius);

            // Suspension distance.
            wc.suspensionDistance = EditorGUILayout.FloatField("Suspension Distance (m)", wc.suspensionDistance);

            // Suspension Spring.
            bool isOpen = EditorGUILayout.Foldout(foldout, "Suspension Spring");
            if (foldout != isOpen)
                foldout = isOpen;

            if (isOpen)
            {
                EditorGUI.indentLevel++;
                var spring = EditorGUILayout.FloatField("Spring (N/m)", wc.suspensionSpring.spring);
                var damper = EditorGUILayout.FloatField("Damper (N¥s/m)", wc.suspensionSpring.damper);
                var targetPosition = EditorGUILayout.FloatField("Target Position", wc.suspensionSpring.targetPosition);

                var suspensionSpring = new JointSpring();
                suspensionSpring.spring = spring;
                suspensionSpring.damper = damper;
                suspensionSpring.targetPosition = targetPosition;

                wc.suspensionSpring = suspensionSpring;
            }

            if (EditorGUI.EndChangeCheck())
            {
                foreach (var t in targets)
                {
                    var otherWheelColliderConfig = t as WheelColliderConfig;
                    var otherWc = otherWheelColliderConfig.WheelCollider;

                    otherWc.radius = wc.radius;
                    otherWc.suspensionDistance = wc.suspensionDistance;
                    otherWc.suspensionSpring = wc.suspensionSpring;
                }
            }
        }

        public void OnDestroy()
        {
            // Enable WheelCollider inspector operation when WheelColliderConfig component is deleted.
            foreach (var t in targets)
            {
                var wheelColliderConfig = t as WheelColliderConfig;

                if (wheelColliderConfig.WheelCollider != null)
                    wheelColliderConfig.WheelCollider.hideFlags = HideFlags.None;
            }
        }
    }
}