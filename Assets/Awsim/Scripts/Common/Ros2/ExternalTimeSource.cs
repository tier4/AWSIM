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
    /// External ROS TimeSource.
    /// </summary>
    public class ExternalTimeSource : ITimeSource
    {
        int _seconds = 0;
        uint _nanoseconds = 0;

        public void Initialize()
        {
            _seconds = 0;
            _nanoseconds = 0;
        }

        public void Dispose()
        {

        }

        public void GetTime(out int seconds, out uint nenoseconds)
        {
            seconds = this._seconds;
            nenoseconds = this._nanoseconds;
        }

        public void SetTime(int seconds, uint nanoseconds)
        {
            this._seconds = seconds;
            this._nanoseconds = nanoseconds;
        }
    }
}
