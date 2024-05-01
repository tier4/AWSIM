using System;
using System.Threading;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// A thread-safe timesource class that provides simulation time based on the dot net system utc time.
    /// This time source takes into account the value of the simulation timescale and
    /// starts at zero value when the simulation is started.
    /// </summary>
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
