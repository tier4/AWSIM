using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Threading;
using ROS2;

namespace AWSIM
{
    public class DotNetSystemTimeSource : ITimeSource
    {
        private DateTime prevDateTime;
        private double time;

        private static readonly object lockObject = new object();


        public DotNetSystemTimeSource()
        {
            prevDateTime = DateTime.Now;
            time = 0.0;
        }

        public void GetTime(out int seconds, out uint nanoseconds)
        {
            lock (lockObject)
            {
                TimeSpan timeSpan = DateTime.Now - prevDateTime;
                prevDateTime = DateTime.Now;

                time += timeSpan.TotalMilliseconds * 0.001f * TimeScaleProvider.TimeScale;
                TimeUtils.TimeFromTotalSeconds(time, out seconds, out nanoseconds);
            }

        }
    }

}
