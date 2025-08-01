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
    [System.Serializable]
    public class GeoCoordinate
    {
        public double Latitude => _latitude;
        public double Longitude => _longitude;
        public double Altitude => _altitude;

        [SerializeField] double _latitude;
        [SerializeField] double _longitude;
        [SerializeField] double _altitude;

        public GeoCoordinate()
        {
            _latitude = 0;
            _longitude = 0;
            _altitude = 0;
        }

        public GeoCoordinate(double latitude, double longitude, double altitude)
        {
            this._latitude = latitude;
            this._longitude = longitude;
            this._altitude = altitude;
        }
    }
}
