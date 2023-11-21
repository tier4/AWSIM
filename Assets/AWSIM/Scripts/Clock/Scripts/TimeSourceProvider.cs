using ROS2;
using System;

namespace AWSIM
{
    /// <summary>
    /// Static class which provide Time Source object of type TimeSourceSelector.TimeSourceType.
    /// </summary>
    public static class TimeSourceProvider
    {
        public enum TimeSourceType
        {
            UNITY,
            SS2
        }

        #region [Event]

        public static event Action onTimeSourceChanged;

        #endregion

        #region [Variable]

        private static ITimeSource currentTimeSource = null;

        private static bool isInitalized = false;

        #endregion

        #region [Life Cycle]

        public static void Initialize()
        {
            currentTimeSource = new UnityTimeSource();  // default time source
            isInitalized = true;
        }

        public static void Dispose()
        {
            isInitalized = false;
            currentTimeSource = null;
        }

        #endregion

        #region [Public Methods]

        /// <summary>
        /// Returns current TimeSource.
        /// In case of current TimeSource equal to null, default TimeSource will be created.
        /// </summary>
        /// <returns>Current ITimeSource.</returns>
        public static ITimeSource GetTimeSource()
        {
            // lazy initialization
            if(!isInitalized)
            {
                Initialize();
            }

            // default time source
            if(currentTimeSource == null)
            {
                currentTimeSource = new UnityTimeSource();
            }

            return currentTimeSource;
        }


        /// <summary>
        /// If the current time source differ from requested, new time source
        /// will be instantiated and event onTimeSource Changed dispatched.
        /// </summary>
        /// <param name="type">Type of requested time source.</param>
        public static void SetTimeSource(TimeSourceType type)
        {
            // lazy initialization
            if(!isInitalized)
            {
                Initialize();
            }

            // ss2 time source
            if(type == TimeSourceType.SS2)
            {
                if(currentTimeSource == null || !(currentTimeSource is ExternalTimeSource))
                {
                    currentTimeSource = new ExternalTimeSource();
                    onTimeSourceChanged?.Invoke();
                }

                return;
            }

            // default time source
            if(currentTimeSource == null || !(currentTimeSource is UnityTimeSource))
            {
                currentTimeSource = new UnityTimeSource();
                onTimeSourceChanged?.Invoke();
            }
        }

        #endregion
    }

}
