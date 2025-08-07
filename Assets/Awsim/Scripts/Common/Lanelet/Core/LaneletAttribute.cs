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
using System.Collections.Generic;
using UnityEngine;

namespace Awsim.Common
{
    /// <summary>
    /// Predefined keys of OSM tags.
    /// Use these keys for obtaining values from <see cref="LaneletAttribute"/>.
    /// </summary>
    public static class LaneletAttributeKey
    {
        public const string Area = "area";
        public const string Type = "type";
        public const string Subtype = "subtype";
        public const string Location = "location";
        public const string SpeedLimit = "speed_limit";
        public const string Height = "height";
        public const string TurnDirection = "turn_direction";
    }

    /// <summary>
    /// Predefined values of OSM tags.
    /// Use these values for distinguishing values obtained from <see cref="LaneletAttribute"/>.
    /// </summary>
    public static class LaneletAttributeValue
    {
        // Types and SubTypes
        public const string Lanelet = "lanelet";
        public const string RegulatoryElement = "regulatory_element";
        public const string StopLine = "stop_line";
        public const string TrafficLight = "traffic_light";
        public const string RightOfWay = "right_of_way";
        public const string TrafficSign = "traffic_sign";

        // Locations
        public const string Urban = "urban";
        public const string Road = "road";

        // Turn directions
        public const string Left = "left";
        public const string Right = "right";
        public const string Straight = "straight";
    }

    /// <summary>
    /// A set of tags of a lanelet primitive.
    /// </summary>
    [Serializable]
    public class LaneletAttribute
    {
        [SerializeField]
        private Tag[] _data;

        /// <summary>
        /// Get a tag value searched by <paramref name="key"/>.
        /// </summary>
        /// <param name="key"></param>
        /// <returns>Tag value</returns>
        public string this[string key]
        {
            get
            {
                for (int i = 0; i < _data.Length; ++i)
                {
                    if (_data[i].Key == key)
                    {
                        return _data[i].Value;
                    }
                }
                throw new KeyNotFoundException();
            }
        }

        /// <summary>
        /// Instantiate <see cref="LaneletAttribute"/>.
        /// </summary>
        /// <param name="tags"></param>
        public LaneletAttribute(Tag[] tags)
        {
            this._data = tags;
        }

        /// <summary>
        /// Search tags and set <paramref name="value"/> to a found value.
        /// </summary>
        /// <param name="key"></param>
        /// <param name="value"></param>
        /// <returns>True if a tag with <paramref name="key"/> is found. False if no tags with with <paramref name="key"/> are found.</returns>
        public bool TryGetValue(string key, out bool value) =>
            TryGetValue(key, t => t == "true", out value);

        /// <summary>
        /// Search tags and set <paramref name="value"/> to a found value.
        /// </summary>
        /// <param name="key"></param>
        /// <param name="value"></param>
        /// <returns>True if a tag with <paramref name="key"/> is found. False if no tags with with <paramref name="key"/> are found.</returns>
        public bool TryGetValue(string key, out float value) =>
            TryGetValue(key, float.Parse, out value);

        /// <summary>
        /// Search tags and set <paramref name="value"/> to a found value.
        /// </summary>
        /// <param name="key"></param>
        /// <param name="value"></param>
        /// <returns>True if a tag with <paramref name="key"/> is found. False if no tags with with <paramref name="key"/> are found.</returns>
        public bool TryGetValue(string key, out string value) =>
            TryGetValue(key, x => x, out value);

        public bool TryGetLocalPosition(out Vector3 position)
        {
            try
            {
                position = new Vector3
                {
                    x = float.Parse(this["local_x"]),
                    y = float.Parse(this["local_y"]),
                    z = float.Parse(this["ele"])
                };
                return true;
            }
            catch
            {
                position = new Vector3();
                return false;
            }
        }

        private bool TryGetValue<T>(string key, Func<string, T> func, out T value)
        {
            try
            {
                value = func(this[key]);
                return true;
            }
            catch
            {
                value = default;
                return false;
            }
        }
    }
}
