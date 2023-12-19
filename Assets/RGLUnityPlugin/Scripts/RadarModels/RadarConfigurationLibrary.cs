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
            MinRange = 1.3f,
            MaxRange = 130.0f,
            MinAzimuthAngle = -65.0f,
            MaxAzimuthAngle = 65.0f,
            MinElevationAngle = -7.5f,
            MaxElevationAngle = 7.5f,
            RangeSeparation = 1.3f,
            AzimuthSeparation = 8.0f,
            NoiseParams = RadarConfiguration.TypicalNoiseParams,
        };

        public static RadarConfiguration SmartmicroDRVEGRD169MediumRange => new RadarConfiguration
        {
            MinRange = 0.6f,
            MaxRange = 56.0f,
            MinAzimuthAngle = -65.0f,
            MaxAzimuthAngle = 65.0f,
            MinElevationAngle = -7.5f,
            MaxElevationAngle = 7.5f,
            RangeSeparation = 0.6f,
            AzimuthSeparation = 8.0f,
            NoiseParams = RadarConfiguration.TypicalNoiseParams,
        };

        public static RadarConfiguration SmartmicroDRVEGRD169ShortRange => new RadarConfiguration
        {
            MinRange = 0.2f,
            MaxRange = 19.0f,
            MinAzimuthAngle = -65.0f,
            MaxAzimuthAngle = 65.0f,
            MinElevationAngle = -7.5f,
            MaxElevationAngle = 7.5f,
            RangeSeparation = 0.2f,
            AzimuthSeparation = 8.0f,
            NoiseParams = RadarConfiguration.TypicalNoiseParams,
        };

        public static RadarConfiguration SmartmicroDRVEGRD169UltraShortRange => new RadarConfiguration
        {
            MinRange = 0.1f,
            MaxRange = 9.5f,
            MinAzimuthAngle = -70.0f,
            MaxAzimuthAngle = 70.0f,
            MinElevationAngle = -14.0f,
            MaxElevationAngle = 14.0f,
            RangeSeparation = 0.1f,
            AzimuthSeparation = 8.0f,
            NoiseParams = RadarConfiguration.TypicalNoiseParams,
        };
    }
}
