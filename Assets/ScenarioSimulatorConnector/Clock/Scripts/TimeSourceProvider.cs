using UnityEngine;
using ROS2;

namespace AWSIM
{
    public class TimeSourceProvider : MonoBehaviour
    {
        public enum SourceType
        {
            UNITY,
            SS2
        }

        #region [Settings]

        [Header("Settings")]
        public SourceType ClockSourceType = SourceType.UNITY;

        #endregion

        #region [Mutable]

        private ITimeSource currentTimeSource = null;

        #endregion

        #region [Public Methods]

        public ITimeSource GetTimeSource()
        {
            if(ClockSourceType == SourceType.SS2)
            {
                if(currentTimeSource == null || !(currentTimeSource is ExternalTimeSource))
                {
                    currentTimeSource = new ExternalTimeSource();
                    return currentTimeSource;
                }

                return currentTimeSource;
            }

            // default time source
            if(currentTimeSource == null || !(currentTimeSource is UnityTimeSource))
            {
                currentTimeSource = new UnityTimeSource();
                return currentTimeSource;
            }

            return currentTimeSource;

        }

        #endregion
    }

}
