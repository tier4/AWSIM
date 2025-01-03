using UnityEngine;
using UnityEditor;

namespace AWSIM
{
    /// <summary>
    /// Editor extension for camera sensor feature manager class.
    /// </summary>
    [CustomEditor(typeof(CameraSensorFeatureManager))]
    public class CameraSensorFeatureManagerEditor : Editor
    {
        private bool debugMode = false;
        private bool prefabSettings = false;

        private SerializedProperty hueProperty;
        private SerializedProperty saturationProperty;
        private SerializedProperty contrastProperty;
        private SerializedProperty postExposureProperty;
        private SerializedProperty sharpnessProperty;

        private SerializedProperty exposureModeProperty;

        private SerializedProperty bloomEffectProperty;
        private SerializedProperty chromaticAberrationProperty;
        private SerializedProperty depthOfFieldProperty;
        private SerializedProperty motionBlurProperty;

        private SerializedProperty targetRendererProperty;

        private void OnEnable() 
        {
            hueProperty = serializedObject.FindProperty("hue");
            saturationProperty = serializedObject.FindProperty("saturation");
            contrastProperty = serializedObject.FindProperty("contrast");
            postExposureProperty =serializedObject.FindProperty("postExposure");
            sharpnessProperty = serializedObject.FindProperty("sharpness");

            exposureModeProperty = serializedObject.FindProperty("exposureMode");   

            bloomEffectProperty = serializedObject.FindProperty("bloomEffect");
            chromaticAberrationProperty = serializedObject.FindProperty("chromaticAberration");
            depthOfFieldProperty = serializedObject.FindProperty("depthOfField");
            motionBlurProperty = serializedObject.FindProperty("motionBlur");

            targetRendererProperty = serializedObject.FindProperty("prefabTargetRenderer");
        }

        public override void OnInspectorGUI()
        {
            // ------    Components    ------- //

            debugMode = GUILayout.Toggle(debugMode, new GUIContent("Show Components", "Debug mode to see the references to the required components"), "Button");

            if(debugMode)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("camera"), true);
                EditorGUILayout.PropertyField(serializedObject.FindProperty("cameraSensor"), true);
                EditorGUILayout.PropertyField(serializedObject.FindProperty("volume"), true);
                EditorGUILayout.PropertyField(serializedObject.FindProperty("profile"), true);
            }
        
            // ------    Prefab Settings    ------- //
            prefabSettings = GUILayout.Toggle(prefabSettings, new GUIContent("Show Prefab Settings", "Section with prefab settings"), "Button");
            
            if(prefabSettings)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("prefabTargetRenderer"), true);
            }

            EditorGUILayout.Space(10f);

            // ------    Image Adjustment    ------- //
            EditorGUILayout.PropertyField(serializedObject.FindProperty("hue"), true);
            if(hueProperty.boolValue)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("hueValue"), true);
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("saturation"), true);
            if(saturationProperty.boolValue)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("saturationValue"), true);
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("contrast"), true);
            if(contrastProperty.boolValue)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("contrastValue"), true);
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("postExposure"), true);
            if(postExposureProperty.boolValue)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("postExposureValue"), true);
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("sharpness"), true);
            if(sharpnessProperty.boolValue)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("sharpnessValue"), true);
            }

            // Show exposure settings only if the prefab is intended for HDRP. 
            if(targetRendererProperty.enumValueIndex == 0)
            {
                // ------    Exposure Settings    ------- //
                EditorGUILayout.PropertyField(serializedObject.FindProperty("exposureMode"), true);   
                if(exposureModeProperty.enumValueIndex != 0)
                {
                    EditorGUILayout.PropertyField(serializedObject.FindProperty("ISO"), true);
                    EditorGUILayout.PropertyField(serializedObject.FindProperty("shutterSpeed"), true);
                    EditorGUILayout.PropertyField(serializedObject.FindProperty("aperture"), true);
                }
            }

            // ------   Distortion Correction    ------- //
            EditorGUILayout.PropertyField(serializedObject.FindProperty("distortionCorrection"), true);

            // ------   Additonal Effect    ------- //
            EditorGUILayout.PropertyField(serializedObject.FindProperty("bloomEffect"), true);
            if(bloomEffectProperty.boolValue)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("bloomValue"), true);
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("chromaticAberration"), true);
            if(chromaticAberrationProperty.boolValue)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("chromaticAberrationValue"), true);
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("depthOfField"), true);
            if(depthOfFieldProperty.boolValue)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("focusDistance"), true);
            
                // Show additional parameters only if the prefab is intended for URP. 
                if(targetRendererProperty.enumValueIndex == 1)
                {
                    EditorGUILayout.PropertyField(serializedObject.FindProperty("aperture"), true);
                }
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("motionBlur"), true);
            if(motionBlurProperty.boolValue)
            {
                // Show additional parameters only if the prefab is intended for URP. 
                if(targetRendererProperty.enumValueIndex == 1)
                {
                    EditorGUILayout.PropertyField(serializedObject.FindProperty("shutterSpeed"), true);
                }
            }

            serializedObject.ApplyModifiedProperties();
        }
    }


}

