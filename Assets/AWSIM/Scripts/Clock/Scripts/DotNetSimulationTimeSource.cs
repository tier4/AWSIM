using System;
using System.Threading;
using ROS2;

namespace AWSIM
{

    public class DotNetSimulationTimeSource : ITimeSource
    {
        private DateTime prevDateTime;
        private double time;
        private bool hasStarted = false;

        private readonly object lockObject = new object();

        public DotNetSimulationTimeSource()
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
                    time = 0.0;

                    prevDateTime = currDateTime;
                }

                TimeSpan timeSpan = currDateTime - prevDateTime;
                prevDateTime = currDateTime;

                time += timeSpan.TotalMilliseconds * 0.001f * TimeScaleProvider.TimeScale;
                TimeUtils.TimeFromTotalSeconds(time, out seconds, out nanoseconds);
            }
        }
    }

}
