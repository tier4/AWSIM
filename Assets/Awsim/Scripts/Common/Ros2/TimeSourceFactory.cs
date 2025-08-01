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

using ROS2;

namespace Awsim.Common
{
    /// <summary>
    /// Factory class for TimeSource.
    /// </summary>
    public static class TimeSourceFactory
    {
        /// <summary>
        /// Generate each TimeSource from the selected TimeSourceType.
        /// </summary>
        /// <param name="timeSourceType">Type of time source to be generated.</param>
        /// <returns></returns>
        public static ITimeSource GetTimeSource(TimeSourceType timeSourceType)
        {
            if (timeSourceType == TimeSourceType.Unity)
            {
                return new UnityTimeSource();
            }
            else if (timeSourceType == TimeSourceType.External)
            {
                return new ExternalTimeSource();
            }
            else if (timeSourceType == TimeSourceType.DotnetSystem)
            {
                return new DotNetSystemTimeSource();
            }
            else if (timeSourceType == TimeSourceType.DotnetSimulation)
            {
                return new DotNetSimulationTimeSource();
            }
            else
            {
                return new ROS2TimeSource();
            }
        }
    }
}