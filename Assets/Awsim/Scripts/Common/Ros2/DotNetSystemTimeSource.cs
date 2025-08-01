// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using ROS2;

namespace Awsim.Common
{
    /// <summary>
    /// A thread-safe timesource class that provides the dot net system utc time since epoch.
    /// This timesource takes into account the value of the simulation timescale.
    /// </summary>
    public class DotNetSystemTimeSource : ITimeSource
    {
        DateTime _prevDateTime;
        double _time;
        bool _hasStarted = false;

        readonly object _lockObject = new object();

        public DotNetSystemTimeSource()
        {
            _hasStarted = false;
        }

        public void GetTime(out int seconds, out uint nanoseconds)
        {
            lock (_lockObject)
            {
                DateTime currDateTime = DateTime.UtcNow;

                if (!_hasStarted)
                {
                    _hasStarted = true;

                    // get the time in millisecond since epoch
                    long timeOffset = ((DateTimeOffset)currDateTime).ToUnixTimeMilliseconds();
                    _time = (double)timeOffset * 0.001;

                    _prevDateTime = currDateTime;
                }

                TimeSpan timeSpan = currDateTime - _prevDateTime;
                _prevDateTime = currDateTime;

                _time += timeSpan.TotalMilliseconds * 0.001 * ThreadSafeTime.TimeScale;
                TimeUtility.TimeFromTotalSeconds(_time, out seconds, out nanoseconds);
            }
        }
    }

}
