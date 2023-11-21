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

    }

}
