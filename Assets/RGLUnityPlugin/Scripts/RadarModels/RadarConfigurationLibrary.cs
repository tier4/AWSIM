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

using System;
using System.Collections.Generic;

namespace RGLUnityPlugin
{
    public static class RadarConfigurationLibrary
    {
        public static readonly Dictionary<RadarModel, Func<RadarConfiguration>> ByModel =
            new Dictionary<RadarModel, Func<RadarConfiguration>>
            {
                {RadarModel.SmartmicroDRVEGRD169LongRange, () => new RadarConfiguration
                {
                    minRange = 1.3f,
                    maxRange = 130.0f,
                    minAzimuthAngle = -65.0f,
                    maxAzimuthAngle = 65.0f,
                    minElevationAngle = -7.5f,
                    maxElevationAngle = 7.5f,
                    separations = new[]
                    {
                        new RadarSeparations
                        {
                            minRange = 1.3f,
                            maxRange = 19.0f,
                            distanceSeparation = 0.3f,
                            speedSeparation = 0.3f,
                            azimuthSeparation = 8.0f
                        },
                        new RadarSeparations
                        {
                            minRange = 19.0f,
                            maxRange = 56.0f,
                            distanceSeparation = 0.6f,
                            speedSeparation = 0.3f,
                            azimuthSeparation = 8.0f
                        },
                        new RadarSeparations
                        {
                            minRange = 56.0f,
                            maxRange = 130.0f,
                            distanceSeparation = 1.3f,
                            speedSeparation = 0.15f,
                            azimuthSeparation = 8.0f
                        }
                    },
                    noiseParams = RadarNoiseParams.TypicalNoiseParams,
                }},

                {RadarModel.SmartmicroDRVEGRD169MediumRange, () => new RadarConfiguration
                {
                    minRange = 0.6f,
                    maxRange = 56.0f,
                    minAzimuthAngle = -65.0f,
                    maxAzimuthAngle = 65.0f,
                    minElevationAngle = -7.5f,
                    maxElevationAngle = 7.5f,
                    separations = new[]
                    {
                        new RadarSeparations
                        {
                            minRange = 0.6f,
                            maxRange = 9.5f,
                            distanceSeparation = 0.15f,
                            speedSeparation = 0.15f,
                            azimuthSeparation = 8.0f
                        },
                        new RadarSeparations
                        {
                            minRange = 9.5f,
                            maxRange = 19.0f,
                            distanceSeparation = 0.3f,
                            speedSeparation = 0.3f,
                            azimuthSeparation = 8.0f
                        },
                        new RadarSeparations
                        {
                            minRange = 9.5f,
                            maxRange = 56.0f,
                            distanceSeparation = 0.6f,
                            speedSeparation = 0.3f,
                            azimuthSeparation = 8.0f
                        },
                    },
                    noiseParams = RadarNoiseParams.TypicalNoiseParams,
                }},

                {RadarModel.SmartmicroDRVEGRD169ShortRange, () => new RadarConfiguration
                {
                    minRange = 0.2f,
                    maxRange = 19.0f,
                    minAzimuthAngle = -65.0f,
                    maxAzimuthAngle = 65.0f,
                    minElevationAngle = -7.5f,
                    maxElevationAngle = 7.5f,
                    separations = new[]
                    {
                        new RadarSeparations
                        {
                            minRange = 0.2f,
                            maxRange = 9.5f,
                            distanceSeparation = 0.15f,
                            speedSeparation = 0.15f,
                            azimuthSeparation = 8.0f
                        },
                        new RadarSeparations
                        {
                            minRange = 9.5f,
                            maxRange = 19.0f,
                            distanceSeparation = 0.3f,
                            speedSeparation = 0.3f,
                            azimuthSeparation = 8.0f
                        },
                    },
                    noiseParams = RadarNoiseParams.TypicalNoiseParams,
                }},

                {RadarModel.SmartmicroDRVEGRD169UltraShortRange, () => new RadarConfiguration
                {
                    minRange = 0.1f,
                    maxRange = 9.5f,
                    minAzimuthAngle = -70.0f,
                    maxAzimuthAngle = 70.0f,
                    minElevationAngle = -14.0f,
                    maxElevationAngle = 14.0f,
                    separations = new[]
                    {
                        new RadarSeparations
                        {
                            minRange = 0.1f,
                            maxRange = 9.5f,
                            distanceSeparation = 0.15f,
                            speedSeparation = 0.15f,
                            azimuthSeparation = 8.0f
                        },
                    },
                    noiseParams = RadarNoiseParams.TypicalNoiseParams,
                }}
            };
    }
}
