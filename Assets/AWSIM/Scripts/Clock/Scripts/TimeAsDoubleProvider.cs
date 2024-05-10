using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// A thread-safe static class to provide the value of the Unity time as double.
    /// </summary>
    public static class TimeAsDoubleProvider
    {
        private static readonly object lockObject = new object();

        private static double timeAsDouble = 0.0;
        public static double TimeAsDouble
        {
            get
            {
                lock(lockObject)
                {
                    return timeAsDouble;
                }
            }
        }


        #region [Public Methods]

        /// <summary>
        /// Synchronise the value of the timeAsDouble variable with the Time.timeAsDouble from the Unity thread.
        /// </summary>
        public static void DoUpdate()
        {
            lock(lockObject)
            {
                timeAsDouble = Time.timeAsDouble;
            }
        }

        #endregion
    }
}

