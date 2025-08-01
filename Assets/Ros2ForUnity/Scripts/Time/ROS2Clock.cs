// Copyright 2019-2022 Robotec.ai.
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
using UnityEngine;

namespace ROS2
{

/// <summary>
/// A ros2 clock class that for interfacing between a time source (unity or ros2 system time) and ros2cs messages, structs. 
/// </summary>
public class ROS2Clock
{
    private ITimeSource _timeSource;

    public ROS2Clock() : this(new ROS2TimeSource())
    {   // By default, use ROS2TimeSource
    }

    public ROS2Clock(ITimeSource ts)
    {
        _timeSource = ts;
    }

    public void UpdateClockMessage(ref rosgraph_msgs.msg.Clock clockMessage)
    {
        int seconds;
        uint nanoseconds;
        _timeSource.GetTime(out seconds, out nanoseconds);
        clockMessage.Clock_.Sec = seconds;
        clockMessage.Clock_.Nanosec = nanoseconds;
    }

    public void UpdateROSClockTime(builtin_interfaces.msg.Time time)
    {
        int seconds;
        uint nanoseconds;
        _timeSource.GetTime(out seconds, out nanoseconds);
        time.Sec = seconds;
        time.Nanosec = nanoseconds;
    }

    public void UpdateROSTimestamp(ref ROS2.MessageWithHeader message)
    {
        int seconds;
        uint nanoseconds;
        _timeSource.GetTime(out seconds, out nanoseconds);
        message.UpdateHeaderTime(seconds, nanoseconds);
    }
}

}  // namespace ROS2
