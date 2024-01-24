using System;
using System.Threading;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// A thread-safe timesource class that provides the dot net system utc time.
    /// </summary>
    public class DotNetSystemTimeSource : ITimeSource
    {
        private DateTime prevDateTime;
        private double time;

        private readonly object lockObject = new object();

        public DotNetSystemTimeSource()
        {
            prevDateTime = DateTime.UtcNow;
            time = 0.0;
        }

        public void GetTime(out int seconds, out uint nanoseconds)
        {
            lock (lockObject)
            {
                DateTime currDateTime = DateTime.UtcNow;
                TimeSpan timeSpan = currDateTime - prevDateTime;
                prevDateTime = currDateTime;

                time += timeSpan.TotalMilliseconds * 0.001f * TimeScaleProvider.TimeScale;
                TimeUtils.TimeFromTotalSeconds(time, out seconds, out nanoseconds);
            }
        }
    }

}
