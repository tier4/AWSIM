using System;
using System.Threading;
using ROS2;

namespace AWSIM
{
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
                TimeSpan timeSpan = DateTime.UtcNow - prevDateTime;
                prevDateTime = DateTime.UtcNow;

                time += timeSpan.TotalMilliseconds * 0.001f * TimeScaleProvider.TimeScale;
                TimeUtils.TimeFromTotalSeconds(time, out seconds, out nanoseconds);
            }
        }
    }

}
