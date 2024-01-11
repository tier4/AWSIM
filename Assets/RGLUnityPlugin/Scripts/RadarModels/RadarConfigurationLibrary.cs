// Copyright 2023 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System.Collections.Generic;

namespace RGLUnityPlugin
{
    public static class RadarConfigurationLibrary
    {
        public static readonly Dictionary<RadarModel, RadarConfiguration> ByModel =
            new Dictionary<RadarModel, RadarConfiguration>
            {
                {RadarModel.SmartmicroDRVEGRD169LongRange, SmartmicroDRVEGRD169LongRange},
                {RadarModel.SmartmicroDRVEGRD169MediumRange, SmartmicroDRVEGRD169MediumRange},
                {RadarModel.SmartmicroDRVEGRD169ShortRange, SmartmicroDRVEGRD169ShortRange},
                {RadarModel.SmartmicroDRVEGRD169UltraShortRange, SmartmicroDRVEGRD169UltraShortRange},
            };

        public static RadarConfiguration SmartmicroDRVEGRD169LongRange => new RadarConfiguration
        {
            minRange = 1.3f,
            maxRange = 130.0f,
            minAzimuthAngle = -65.0f,
            maxAzimuthAngle = 65.0f,
            minElevationAngle = -7.5f,
            maxElevationAngle = 7.5f,
            rangeSeparation = 1.3f,
            azimuthSeparation = 8.0f,
            noiseParams = RadarConfiguration.TypicalNoiseParams,
        };

        public static RadarConfiguration SmartmicroDRVEGRD169MediumRange => new RadarConfiguration
        {
            minRange = 0.6f,
            maxRange = 56.0f,
            minAzimuthAngle = -65.0f,
            maxAzimuthAngle = 65.0f,
            minElevationAngle = -7.5f,
            maxElevationAngle = 7.5f,
            rangeSeparation = 0.6f,
            azimuthSeparation = 8.0f,
            noiseParams = RadarConfiguration.TypicalNoiseParams,
        };

        public static RadarConfiguration SmartmicroDRVEGRD169ShortRange => new RadarConfiguration
        {
            minRange = 0.2f,
            maxRange = 19.0f,
            minAzimuthAngle = -65.0f,
            maxAzimuthAngle = 65.0f,
            minElevationAngle = -7.5f,
            maxElevationAngle = 7.5f,
            rangeSeparation = 0.2f,
            azimuthSeparation = 8.0f,
            noiseParams = RadarConfiguration.TypicalNoiseParams,
        };

        public static RadarConfiguration SmartmicroDRVEGRD169UltraShortRange => new RadarConfiguration
        {
            minRange = 0.1f,
            maxRange = 9.5f,
            minAzimuthAngle = -70.0f,
            maxAzimuthAngle = 70.0f,
            minElevationAngle = -14.0f,
            maxElevationAngle = 14.0f,
            rangeSeparation = 0.1f,
            azimuthSeparation = 8.0f,
            noiseParams = RadarConfiguration.TypicalNoiseParams,
        };
    }
}
