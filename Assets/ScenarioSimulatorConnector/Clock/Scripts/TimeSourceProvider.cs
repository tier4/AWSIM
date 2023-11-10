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
            if(timeSourceSelector != null)
            {
                currentTimeSource = null;
                isInitalized = true;
            }
            else
            {
                UnityEngine.Debug.LogError("TimeSourceProvider requires TimeSourceSelector object to be presented on the scene. Check if TimeSourceSelector is on the scene.");
            }
        }

        public static void Dispose()
        {
            isInitalized = false;
            timeSourceSelector = null;
            currentTimeSource = null;
        }

        #endregion

        #region [Public Methods]

        public static ITimeSource GetTimeSource()
        {
            // lazy initialization
            if(!isInitalized)
            {
                Initialize();
            }

            if(timeSourceSelector.Type == TimeSourceSelector.TimeSourceType.SS2)
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
