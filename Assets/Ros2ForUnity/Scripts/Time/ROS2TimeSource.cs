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

using UnityEngine;

namespace ROS2
{

/// <summary>
/// ros2 time source (system time by default).
/// </summary>
public class ROS2TimeSource : ITimeSource
{
  private ROS2.Clock clock;

  public void GetTime(out int seconds, out uint nanoseconds)
  {
    if (!ROS2.Ros2cs.Ok())
    {
      seconds = 0;
      nanoseconds = 0;
      Debug.LogWarning("Cannot acquire valid ros time, ros either not initialized or shut down already");
      return;
    }

    if (clock == null)
    { // Create clock which uses system time by default (unless use_sim_time is set in ros2)
      clock = new ROS2.Clock();
    }
  
    TimeUtils.TimeFromTotalSeconds(clock.Now.Seconds, out seconds, out nanoseconds);
  }

  ~ROS2TimeSource()
  {
    if (clock != null)
    {
      clock.Dispose();
    }
  }
}

}  // namespace ROS2
