using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// MonoBehaviour class for selecting active Time Source.
    /// </summary>
    [DefaultExecutionOrder(-1)]
    public class TimeSourceSelector : MonoBehaviour
    {
        #region [Settings]

        [Header("Settings")]
        public TimeSourceProvider.TimeSourceType Type = TimeSourceProvider.TimeSourceType.UNITY;

        #endregion

        private void Awake()
        {
            TimeSourceProvider.SetTimeSource(Type);
        }

        #region [Public Methods]

        /// <summary>
        /// Set the time source of the desired type.
        /// </summary>
        /// <param name="type">Type of requested time source.</param>
        public void SetType(string type)
        {
            if(type == null || type == "")
            {
                return;
            }

            if(string.CompareOrdinal(type.ToLower(), "unity") == 0)
            {
                SetType(TimeSourceProvider.TimeSourceType.UNITY);
            }
            else if(string.CompareOrdinal(type.ToLower(), "simulation") == 0)
            {
                SetType(TimeSourceProvider.TimeSourceType.DOTNET_SIMULATION);
            }
            else if(string.CompareOrdinal(type.ToLower(), "system") == 0)
            {
                SetType(TimeSourceProvider.TimeSourceType.DOTNET);
            }
            else if(string.CompareOrdinal(type.ToLower(), "ss2") == 0)
            {
                SetType(TimeSourceProvider.TimeSourceType.SS2);
            }
            else if(string.CompareOrdinal(type.ToLower(), "ros2") == 0)
            {
                SetType(TimeSourceProvider.TimeSourceType.ROS2);
            }
            else
            {
                Debug.LogWarning("TimeSourceSelector: " + type + " is not recognized as a valid time source.");
            }
        }

        #endregion

        #region [Private Methods]

        /// <summary>
        /// Set the time source of the desired type.
        /// </summary>
        /// <param name="type">Type of requested time source.</param>
        private void SetType(TimeSourceProvider.TimeSourceType type)
        {
            Type = type;
            TimeSourceProvider.SetTimeSource(Type);
        }

        #endregion
    }

}
