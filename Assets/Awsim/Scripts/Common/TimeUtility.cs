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

namespace Awsim.Common
{
    /// <summary>
    /// Utility class for Time.
    /// </summary>
    public static class TimeUtility
    {
        /// <summary>
        /// Split into seconds and nanoseconds from total seconds.
        /// </summary>
        /// <param name="secondIn">total seconds.</param>
        /// <param name="seconds">output seconds.</param>
        /// <param name="nanoseconds">output nano seconds.</param>
        public static void TimeFromTotalSeconds(in double secondIn, out int seconds, out uint nanoseconds)
        {
            long nanosec = (long)(secondIn * 1e9);
            seconds = (int)(nanosec / 1000000000);
            nanoseconds = (uint)(nanosec % 1000000000);
        }
    }
}