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
    /// Utility for unity obejct.
    /// </summary>
    public static class UnityObject
    {
        /// <summary>
        /// Destroy unity object with leaked managed shell.
        /// </summary>
        /// <see cref="https://unity.com/blog/engine-platform/custom-operator-should-we-keep-it"/>
        /// <typeparam name="T"></typeparam>
        /// <param name="obj"></param>
        /// <param name="t"></param>
        public static void DestroySafe<T>(ref T obj, float t = 0) where T : Object
        {
            if (obj != null)
            {
                Object.Destroy(obj, t);
                obj = null;
            }
        }

        /// <summary>
        /// Destroy unity object with leaked managed shell.
        /// </summary>
        /// <see cref="https://unity.com/blog/engine-platform/custom-operator-should-we-keep-it"/>
        /// <typeparam name="T"></typeparam>
        /// <param name="obj"></param>
        public static void DestroySafe<T>(ref T obj) where T : Object
        {
            DestroySafe(ref obj, 0f);
        }
    }
}