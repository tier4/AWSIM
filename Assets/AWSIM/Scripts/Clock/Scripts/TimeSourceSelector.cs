using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// MonoBehaviour class for selecting active Time Source.
    /// </summary>
    [DefaultExecutionOrder(-1)]
    public class TimeSourceSelector : MonoBehaviour
    {
        public enum TimeSourceType
        {
            UNITY,
            SS2
        }

        #region [Settings]

        [Header("Settings")]
        public TimeSourceType Type = TimeSourceType.UNITY;

        #endregion

        private void Awake()
        {
            TimeSourceProvider.SetTimeSource(Type);
        }

    }

}
