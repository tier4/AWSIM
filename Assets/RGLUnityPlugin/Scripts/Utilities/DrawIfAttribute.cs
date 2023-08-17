using System;
using UnityEngine;
using UnityEditor;

namespace RGLUnityPlugin
{
    /// <summary>
    /// Draws the field/property ONLY if the compared property compared by the comparison type with the value of comparedValue returns true.
    /// Based on: https://forum.unity.com/threads/draw-a-field-only-if-a-condition-is-met.448855/#post-3435603
    /// </summary>
    [AttributeUsage(AttributeTargets.Property | AttributeTargets.Field, AllowMultiple = true)]
    public class DrawIfAttribute : PropertyAttribute
    {
        #region Fields

        public string comparedPropertyName { get; private set; }
        public object comparedValue { get; private set; }

        #endregion

        /// <summary>
        /// Only draws the field only if a condition is met. Supports enum and bools.
        /// </summary>
        /// <param name="comparedPropertyName">The name of the property that is being compared (case sensitive).</param>
        /// <param name="comparedValue">The value the property is being compared to.</param>
        public DrawIfAttribute(string comparedPropertyName, object comparedValue)
        {
            this.comparedPropertyName = comparedPropertyName;
            this.comparedValue = comparedValue;
        }
    }

    #if UNITY_EDITOR
    /// <summary>
    /// Based on: https://forum.unity.com/threads/draw-a-field-only-if-a-condition-is-met.448855/#post-3435603
    /// </summary>
    [CustomPropertyDrawer(typeof(DrawIfAttribute))]
    public class DrawIfPropertyDrawer : PropertyDrawer
    {
        #region Fields

        // Reference to the attribute on the property.
        DrawIfAttribute drawIf;
    
        // Field that is being compared.
        SerializedProperty comparedField;

        #endregion

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            if (!ShowMe(property))
            {
                return 0f;
            }

            // The height of the property should be defaulted to the default height.
            return base.GetPropertyHeight(property, label);
        }

        /// <summary>
        /// Errors default to showing the property.
        /// </summary>
        private bool ShowMe(SerializedProperty property)
        {
            drawIf = attribute as DrawIfAttribute;
            // Replace propertyname to the value from the parameter
            string path = property.propertyPath.Contains(".")
                ? System.IO.Path.ChangeExtension(property.propertyPath, drawIf.comparedPropertyName)
                : drawIf.comparedPropertyName;

            comparedField = property.serializedObject.FindProperty(path);

            if (comparedField == null)
            {
                Debug.LogError("Cannot find property with name: " + path);
                return true;
            }

            // Get the value & compare based on types
            switch (comparedField.type)
            { // Possible extend cases to support more types
                case "bool":
                    return comparedField.boolValue.Equals(drawIf.comparedValue);
                case "Enum":
                    return comparedField.enumValueIndex.Equals((int)drawIf.comparedValue);
                default:
                    Debug.LogError("Error: " + comparedField.type + " is not supported of " + path);
                    return true;
            }
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            // If the condition is met, simply draw the field.
            if (ShowMe(property))
            {
                EditorGUI.PropertyField(position, property, label);
            }
        }
    }
    #endif
}
