using UnityEngine;
using UnityEditor;

namespace AWSIM
{

    [CustomEditor(typeof(CameraSensorFeatureManager))]
    public class CameraSensorFeatureManagerEditor : Editor
    {

        private CameraSensorFeatureManager manager;

        private SerializedProperty hueProperty;
        private SerializedProperty saturationProperty;
        private SerializedProperty contrastProperty;
        private SerializedProperty postExposureProperty;

        private SerializedProperty exposureModeProperty;

        private SerializedProperty bloomEffectProperty;
        private SerializedProperty chromaticAberrationProperty;
        private SerializedProperty depthOfFieldProperty;
        private SerializedProperty motionBlurProperty;


        private void OnEnable() 
        {
            manager = (CameraSensorFeatureManager)target;

            hueProperty = serializedObject.FindProperty("hue");
            saturationProperty = serializedObject.FindProperty("saturation");
            contrastProperty = serializedObject.FindProperty("contrast");
            postExposureProperty =serializedObject.FindProperty("postExposure");

            exposureModeProperty = serializedObject.FindProperty("exposureMode");   

            bloomEffectProperty = serializedObject.FindProperty("bloomEffect");
            chromaticAberrationProperty = serializedObject.FindProperty("chromaticAberration");
            depthOfFieldProperty = serializedObject.FindProperty("depthOfField");
            motionBlurProperty = serializedObject.FindProperty("motionBlur");
        }

        public override void OnInspectorGUI()
        {
            // ------    Components    ------- //
            EditorGUILayout.PropertyField(serializedObject.FindProperty("camera"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("cameraSensor"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("volume"), true);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("profile"), true);
        
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

            // ------    Exposure Settings    ------- //
            EditorGUILayout.PropertyField(serializedObject.FindProperty("exposureMode"), true);   
            if(exposureModeProperty.enumValueIndex != 0)
            {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("ISO"), true);
                EditorGUILayout.PropertyField(serializedObject.FindProperty("shutterSpeed"), true);
                EditorGUILayout.PropertyField(serializedObject.FindProperty("aperture"), true);
            }

            
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
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("motionBlur"), true);


            serializedObject.ApplyModifiedProperties();
        }
    }


}

