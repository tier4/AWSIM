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
                    minAzimuthAngle = -65.0f,
                    maxAzimuthAngle = 65.0f,
                    minElevationAngle = -7.5f,
                    maxElevationAngle = 7.5f,
                    scopeParametersList = new[]
                    {
                        new RadarScopeParameters
                        {
                            beginDistance = 1.3f,
                            endDistance = 19.0f,
                            distanceSeparationThreshold = 0.3f,
                            radialSpeedSeparationThreshold = 0.3f,
                            azimuthSeparationThreshold = 8.0f
                        },
                        new RadarScopeParameters
                        {
                            beginDistance = 19.0f,
                            endDistance = 56.0f,
                            distanceSeparationThreshold = 0.6f,
                            radialSpeedSeparationThreshold = 0.3f,
                            azimuthSeparationThreshold = 8.0f
                        },
                        new RadarScopeParameters
                        {
                            beginDistance = 56.0f,
                            endDistance = 130.0f,
                            distanceSeparationThreshold = 1.3f,
                            radialSpeedSeparationThreshold = 0.15f,
                            azimuthSeparationThreshold = 8.0f
                        }
                    },
                    noiseParams = RadarNoiseParams.TypicalNoiseParams,
                }},

                {RadarModel.SmartmicroDRVEGRD169MediumRange, () => new RadarConfiguration
                {
                    minAzimuthAngle = -65.0f,
                    maxAzimuthAngle = 65.0f,
                    minElevationAngle = -7.5f,
                    maxElevationAngle = 7.5f,
                    scopeParametersList = new[]
                    {
                        new RadarScopeParameters
                        {
                            beginDistance = 0.6f,
                            endDistance = 9.5f,
                            distanceSeparationThreshold = 0.15f,
                            radialSpeedSeparationThreshold = 0.15f,
                            azimuthSeparationThreshold = 8.0f
                        },
                        new RadarScopeParameters
                        {
                            beginDistance = 9.5f,
                            endDistance = 19.0f,
                            distanceSeparationThreshold = 0.3f,
                            radialSpeedSeparationThreshold = 0.3f,
                            azimuthSeparationThreshold = 8.0f
                        },
                        new RadarScopeParameters
                        {
                            beginDistance = 19.0f,
                            endDistance = 56.0f,
                            distanceSeparationThreshold = 0.6f,
                            radialSpeedSeparationThreshold = 0.3f,
                            azimuthSeparationThreshold = 8.0f
                        },
                    },
                    noiseParams = RadarNoiseParams.TypicalNoiseParams,
                }},

                {RadarModel.SmartmicroDRVEGRD169ShortRange, () => new RadarConfiguration
                {
                    minAzimuthAngle = -65.0f,
                    maxAzimuthAngle = 65.0f,
                    minElevationAngle = -7.5f,
                    maxElevationAngle = 7.5f,
                    scopeParametersList = new[]
                    {
                        new RadarScopeParameters
                        {
                            beginDistance = 0.2f,
                            endDistance = 9.5f,
                            distanceSeparationThreshold = 0.15f,
                            radialSpeedSeparationThreshold = 0.15f,
                            azimuthSeparationThreshold = 8.0f
                        },
                        new RadarScopeParameters
                        {
                            beginDistance = 9.5f,
                            endDistance = 19.0f,
                            distanceSeparationThreshold = 0.3f,
                            radialSpeedSeparationThreshold = 0.3f,
                            azimuthSeparationThreshold = 8.0f
                        },
                    },
                    noiseParams = RadarNoiseParams.TypicalNoiseParams,
                }},

                {RadarModel.SmartmicroDRVEGRD169UltraShortRange, () => new RadarConfiguration
                {
                    minAzimuthAngle = -70.0f,
                    maxAzimuthAngle = 70.0f,
                    minElevationAngle = -14.0f,
                    maxElevationAngle = 14.0f,
                    scopeParametersList = new[]
                    {
                        new RadarScopeParameters
                        {
                            beginDistance = 0.1f,
                            endDistance = 9.5f,
                            distanceSeparationThreshold = 0.15f,
                            radialSpeedSeparationThreshold = 0.15f,
                            azimuthSeparationThreshold = 8.0f
                        },
                    },
                    noiseParams = RadarNoiseParams.TypicalNoiseParams,
                }}
            };
    }
}
