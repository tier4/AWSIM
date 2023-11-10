using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Static class which provide Time Source object of type chosen in TimeSourceSelector.
    /// </summary>
    public static class TimeSourceProvider
    {
        #region [Variable]

        private static TimeSourceSelector timeSourceSelector = null;

        private static ITimeSource currentTimeSource = null;

        private static bool isInitalized = false;

        #endregion

        #region [Life Cycle]

        public static void Initialize()
        {
            timeSourceSelector = UnityEngine.Object.FindObjectOfType<TimeSourceSelector>();
            currentTimeSource = null;
            isInitalized = true;
        }

        public static void Dispose()
        {
            isInitalized = false;
            timeSourceSelector = null;
            currentTimeSource = null;
        }

        #endregion

        #region [Public Methods]

        /// <summary>
        /// Returns current TimeSource. The type of required TimeSource is selected in TimeSourceSelector object.
        /// In case of scene without TimeSourceSelect object, the method returns default time source.
        /// </summary>
        /// <returns>ITimeSource of type chosen in TimeSourceSelector.</returns>
        public static ITimeSource GetTimeSource()
        {
            // lazy initialization
            if(!isInitalized)
            {
                Initialize();
            }

            // return time source of type from time source selector
            if(timeSourceSelector != null && timeSourceSelector.Type == TimeSourceSelector.TimeSourceType.SS2)
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
