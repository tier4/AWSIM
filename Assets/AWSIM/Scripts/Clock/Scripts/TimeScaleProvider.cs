using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// A thread-safe static class to provide the value of the time scale of the simulation.
    /// </summary>
    public static class TimeScaleProvider
    {
        private static readonly object lockObject = new object();

        private static float timeScale = 1.0f;
        public static float TimeScale
        {
            get
            {
                lock (lockObject)
                {
                    return timeScale;
                }
            }
        }


        #region [Public Methods]

        /// <summary>
        /// Synchronise the value of the timeScale variable with the Time.timeScale from the Unity thread.
        /// </summary>
        public static void DoUpdate()
        {
            lock(lockObject)
            {
                timeScale = Time.timeScale;
            }
        }

        #endregion
    }
}

