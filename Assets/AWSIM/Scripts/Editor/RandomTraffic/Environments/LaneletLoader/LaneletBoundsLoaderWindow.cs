using AWSIM.Lanelet;
using UnityEditor;
using UnityEngine;

namespace AWSIM.RandomTraffic
{
    public class LaneletBoundsLoaderWindow : EditorWindow
    {
        [SerializeField] private OsmDataContainer osm;
        [SerializeField] private LaneletBoundsLoader.WaypointSettings waypointSettings = LaneletBoundsLoader.WaypointSettings.Default();
        private SerializedObject serializedObject;

        [MenuItem("AWSIM/Random Traffic/Load Lanelet Bounds")]
        private static void ShowWindow()
        {
            var window = GetWindow(typeof(LaneletBoundsLoaderWindow), true, "LaneletBoundsLoader");
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

            if (GUILayout.Button("Load"))
            {
                var referencePoint = Environment.Instance.MgrsOffsetPosition;
                var loader = new LaneletBoundsLoader();
                loader.SetWaypointSettings(waypointSettings);
                loader.Load(osm.Data, referencePoint, Environment.Instance.gameObject);
            }
        }
    }
}
