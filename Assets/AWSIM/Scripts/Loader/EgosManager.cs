using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;

namespace AWSIM.Loader 
{
    /// <summary>
    /// Manager for configuring the Ego.
    /// </summary>
    public class EgosManager : MonoBehaviour, IConfigurableManager
    {
        [SerializeField]
        [Tooltip("Available Ego vehicle prefabs.")]
        private GameObject[] egoVehicles;

        private Vector3 egoPosition;

        private Vector3 egoEulerAngles;

        private EgoConfiguration egoConfiguration;

        private GameObject egoModel;

        [Header("GUI")]
        [Tooltip("Dropdown selector for picking the Ego vehicle name.")]
        public Dropdown egoUISelecor;

        [Tooltip("X, Y, Z position GUI input fields.")]
        public InputField[] positionInputFields;

        [Tooltip("Euler angles GUI input fields.")]
        public InputField[] rotationInputFields;

        /// <summary>
        /// Log callback action.
        /// </summary>
        public Action<LogLevel, string> Log  { get; set; }

        /// <summary>
        /// Load and validate config.
        /// </summary>
        public bool LoadConfig(AWSIMConfiguration config)
        {
            this.egoConfiguration = config.egoConfiguration;

            // Validate config
            foreach (var availableEgo in egoVehicles)
            {
                if (egoConfiguration.egoVehicleName == availableEgo.name)
                {
                    egoModel = availableEgo;
                    return true;
                }
            }

            Log(LogLevel.LOG_ERROR, $"No vehicle '{egoConfiguration.egoVehicleName}' available.");
            return false;
        }

        /// <summary>
        /// Set up the UI for Ego configuration
        /// </summary>
        public void LoadUI()
        {
            egoUISelecor.options.Clear();
            foreach (var ego in egoVehicles)
            {
                egoUISelecor.options.Add(
                    new Dropdown.OptionData(
                        ego.name
                    )
                );
            }

            egoUISelecor.value = 0;
            egoUISelecor.RefreshShownValue();
        }

        /// <summary>
        /// Gets the position from the GUI input fields.
        /// </summary>
        public Vector3 GetPositionFromUI()
        {
            return new Vector3() {
                x = float.Parse(positionInputFields[0].text),
                y = float.Parse(positionInputFields[1].text),
                z = float.Parse(positionInputFields[2].text)
            };
        }

        /// <summary>
        /// Gets the euler rotation from GUI input fields.
        /// </summary>
        public Vector3 GetEulersFromUI()
        {
            return new Vector3() {
                x = float.Parse(rotationInputFields[0].text),
                y = float.Parse(rotationInputFields[1].text),
                z = float.Parse(rotationInputFields[2].text)
            };
        }

        private void SpawnEgo(Transform hierarchyParent)
        {
            if (egoModel)
            {
                var position = egoConfiguration.egoPosition;
                position -= Environment.Instance.MgrsOffsetPosition;
                position = ROS2Utility.RosToUnityPosition(position);
                
                var orientation = Quaternion.Euler(egoConfiguration.egoEulerAngles);
                orientation = ROS2Utility.RosToUnityRotation(orientation);

                var egoInstance = Instantiate(egoModel, position, orientation);

                egoInstance.gameObject.transform.SetParent(hierarchyParent);
            } else {
                Log(LogLevel.LOG_ERROR, $"No ego prefab set.");
            }
        }


        /// <summary>
        /// Configure scene
        /// Spawns an Ego.
        /// </summary>
        public void ConfigureScene(Transform rootTransform = null)
        {
            SpawnEgo(rootTransform);
        }

        /// <summary>
        /// Get available Ego vehicle names.
        /// </summary>
        public string[] GetAvailableEgoNames()
        {
            return egoVehicles.Select(x => x.name).ToArray();
        }
    }
}
