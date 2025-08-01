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

using UnityEngine;

namespace Awsim.Common
{
    /// <summary>
    /// Sync unity's time to thread-safe.
    /// </summary>
    public static class ThreadSafeTime
    {
        /// <summary>
        /// Synchronized Time.timeAsDouble.
        /// </summary>
        public static double TimeAsDouble
        {
            get
            {
                lock (_timeAsDoubleLockObject)
                {
                    return _timeAsDouble;
                }
            }
        }

        /// <summary>
        /// Synchronized Time.timeScale.
        /// </summary>
        public static float TimeScale
        {
            get
            {
                lock (_timeScaleLockObject)
                {
                    return _timeScale;
                }
            }
        }

        static readonly object _timeAsDoubleLockObject = new object();
        static readonly object _timeScaleLockObject = new object();
        static double _timeAsDouble = 0.0f;
        static float _timeScale = 1.0f;

        /// <summary>
        /// Synchronize the value of the timeAsDouble variable with the Time.timeAsDouble from the Unity thread.
        /// </summary>
        public static void SyncTimeAsDouble()
        {
            lock (_timeAsDoubleLockObject)
            {
                _timeAsDouble = Time.timeAsDouble;
            }
        }


        /// <summary>
        /// Synchronize the value of the timeScale variable with the Time.timeScale from the Unity thread.
        /// </summary>
        public static void SyncTimeScale()
        {
            lock (_timeScaleLockObject)
            {
                _timeScale = Time.timeScale;
            }
        }
    }
}