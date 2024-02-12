using System;
using System.Threading;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// A thread-safe timesource class that provides the dot net system utc time since epoch.
    /// This timesource takes into account the value of the simulation timescale.
    /// </summary>
    public class DotNetSystemTimeSource : ITimeSource
    {
        private DateTime prevDateTime;
        private double time;
        private bool hasStarted = false;

        private readonly object lockObject = new object();

        public DotNetSystemTimeSource()
        {
            hasStarted = false;
        }

        public void GetTime(out int seconds, out uint nanoseconds)
        {
            lock (lockObject)
            {
                DateTime currDateTime = DateTime.UtcNow;

                if(!hasStarted)
                {
                    hasStarted = true;

                    // get the time in millisecond since epoch
                    long timeOffset = ((DateTimeOffset)currDateTime).ToUnixTimeMilliseconds();
                    time = (double)timeOffset * 0.001;

                    prevDateTime = currDateTime;
                }

                TimeSpan timeSpan = currDateTime - prevDateTime;
                prevDateTime = currDateTime;

                time += timeSpan.TotalMilliseconds * 0.001 * TimeScaleProvider.TimeScale;
                TimeUtils.TimeFromTotalSeconds(time, out seconds, out nanoseconds);
            }
        }
    }

}
