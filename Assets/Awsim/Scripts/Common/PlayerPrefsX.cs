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
    /// PlayerPrefs for other types.
    /// </summary>
    public static class PlayerPrefsX
    {
        /// <summary>
        /// Sets a single boolean value for the preference identified by the given key.
        /// You can use PlayerPrefsUtility.Get boolean to retrieve this value.
        /// </summary>
        /// <param name="key"></param>
        /// <param name="value"></param>
        public static void SetBool(string key, bool value)
        {
            var intValue = value ? 1 : 0;
            UnityEngine.PlayerPrefs.SetInt(key, intValue);
        }

        /// <summary>
        /// Returns the value corresponding to key in the preference file if it exists.
        /// </summary>
        /// <param name="key"></param>
        /// <param name="defaultValue"></param>
        /// <returns></returns>
        public static bool GetBool(string key, bool defaultValue)
        {
            var defaultIntValue = defaultValue ? 1 : 0;
            var resultIntValue = UnityEngine.PlayerPrefs.GetInt(key, defaultIntValue);
            return resultIntValue == 1 ? true : false;
        }

    }
}