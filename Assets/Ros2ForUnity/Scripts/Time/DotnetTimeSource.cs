// Copyright 2022 Robotec.ai.
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
using System.Diagnostics;

namespace ROS2
{

/// <summary>
/// DateTime based clock that has resolution increased using Stopwatch.
/// DateTime is used to synchronize since Stopwatch tends to drift.
/// </summary>
public class DotnetTimeSource : ITimeSource
{
    private readonly double maxUnsyncedSeconds = 10;

    private Stopwatch stopwatch = new Stopwatch();

    private readonly object mutex = new object();

    private double systemTimeIntervalStart = 0;

    private double stopwatchStartTimeStamp;

    private double TotalSystemTimeSeconds()
    {
        return TimeSpan.FromTicks(DateTime.UtcNow.Ticks).TotalSeconds;
    }

    private void UpdateSystemTime()
    {
        systemTimeIntervalStart = TotalSystemTimeSeconds();
        stopwatchStartTimeStamp = Stopwatch.GetTimestamp();
    }

    public DotnetTimeSource()
    {
        UpdateSystemTime();
    }

    public void GetTime(out int seconds, out uint nanoseconds)
    {
        lock(mutex) // Threading
        {
            double endTimestamp = Stopwatch.GetTimestamp();
            var durationInSeconds = endTimestamp - stopwatchStartTimeStamp;
            double timeOffset = 0;
            if (durationInSeconds >= maxUnsyncedSeconds)
            {   // acquire DateTime to sync
                UpdateSystemTime();
            }
            else
            {   // use Stopwatch offset
                timeOffset = durationInSeconds;
            }
            
            TimeUtils.TimeFromTotalSeconds(systemTimeIntervalStart + timeOffset, out seconds, out nanoseconds);
        }
    }
}

}  // namespace ROS2

