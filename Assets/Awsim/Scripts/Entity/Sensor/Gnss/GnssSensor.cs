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
using UnityEngine;
using Awsim.Common;
using GeographicLib;

namespace Awsim.Entity
{
    public enum GnssOutputMode
    {
        Mgrs,
        NavSatFix
    }

    /// <summary>
    /// Global Navigation Satellite System (GNSS) sensor.
    /// Mgrs and GeoCoordinate values are output according to OutputHz.
    /// Need to set the MgrsLocator and GeoCoordinateLocator in scene.
    /// </summary>
    public class GnssSensor : MonoBehaviour
    {
        public interface IReadOnlyOutputData
        {
            public Mgrs Mgrs { get; }
            public GeoCoordinate GeoCoordinate { get; }
        }

        public class OutputData : IReadOnlyOutputData
        {
            public Mgrs Mgrs { get; set; }
            public GeoCoordinate GeoCoordinate { get; set; }

            public OutputData()
            {
                Mgrs = null;                // TODO: Create an instance once with this constructor.
                GeoCoordinate = null;       // TODO: Create an instance once with this constructor.
            }
        }

        /// <summary>
        /// Action used in sensor callbacks.
        /// </summary>
        /// <param name="outputData">Data output for each hz</param>
        public Action<OutputData> OnOutput { get; set; } = null;

        /// <summary>
        /// Data output hz.
        /// Sensor processing and callbacks are called in this hz.
        /// </summary>
        public int OutputHz { get => _outputHz; }

        [SerializeField] int _outputHz = 1;      // Autoware gnss sensor basically output at 1hz.
        [SerializeField] GnssOutputMode _outputMode = GnssOutputMode.Mgrs;
        public GnssOutputMode OutputMode => _outputMode;
        OutputData _outputData = null;
        Transform _transform = null;

        /// <summary>
        /// Initialize gnss sensor.
        /// </summary>
        public void Initialize()
        {
            _outputData = new OutputData();
            _transform = transform;

            InvokeRepeating(nameof(Output), 0f, 1f / OutputHz);
        }

        /// <summary>
        /// Initialize Gnss Sensor.
        /// </summary>
        /// <param name="outputHz">Output cycle of gnss sensor.</param>
        public void Initialize(int outputHz)
        {
            _outputHz = outputHz;

            Initialize();
        }

        /// <summary>
        // We intentionally use MGRS as an intermediate coordinate system.
        //
        // Reason:
        // - Directly using latitude/longitude in Unity/ROS often causes floating-point
        //   precision issues. 
        // - MGRS/UTM provides meter-based local coordinates, which avoids this precision loss
        //   and ensures consistent initialization near the target area.
        //
        // Therefore, the conversion pipeline is:
        // Unity (meters) → ROS (meters) → MGRS (grid-based) → UTM → Latitude/Longitude.
        /// </summary>
        void Output()
        {
            var unityPosition = _transform.position;
            var rosPosition = Ros2Utility.UnityToRos2Position(unityPosition);
            var mgrsBase      = MgrsPosition.Instance.Mgrs;
            var mgrsPosition = rosPosition + mgrsBase.Position;

            _outputData.Mgrs = new Mgrs(mgrsPosition, mgrsBase.GridZone);

            string mgrsString = mgrsBase.GridZone + string.Format("{0:D9}", (int)(mgrsPosition.x * 10000)) + string.Format("{0:D9}", (int)(mgrsPosition.y * 10000)); 
            (int utmZone, bool utmNorthp, double utmX, double utmY, int utmPrec) = GeographicLib.Geocodes.MGRS.Reverse(mgrsString.AsSpan());
            (double latDeg, double lonDeg) = GeographicLib.UTMUPS.Reverse(utmZone, utmNorthp, utmX, utmY);

            double height = mgrsPosition.z; 
            _outputData.GeoCoordinate = new GeoCoordinate(latDeg, lonDeg, height);

            // Calls registered callbacks.
            OnOutput?.Invoke(_outputData);
        }
    }
}