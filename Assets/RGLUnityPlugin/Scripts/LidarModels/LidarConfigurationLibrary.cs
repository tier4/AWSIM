// Copyright 2022 Robotec.ai.
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
    public static class LidarConfigurationLibrary
    {
        public static readonly Dictionary<LidarModel, Func<BaseLidarConfiguration>> ByModel =
            new Dictionary<LidarModel, Func<BaseLidarConfiguration>>
            {
                {LidarModel.RangeMeter, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArray.Uniform(-0.0f, 0.0f, 1),
                    horizontalResolution = 1.0f,
                    minHAngle = -0,
                    maxHAngle = 0,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0,
                    maxRange = 40,
                    horizontalBeamDivergence = 0.13f,
                    verticalBeamDivergence = 0.13f,
                }},

                {LidarModel.SickMRS6000, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArray.Uniform(-15f, 1.5f, 24),
                    horizontalResolution = 240.0f / 924.0f,
                    minHAngle = -120,
                    maxHAngle = 120,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0,
                    maxRange = 40,
                    horizontalBeamDivergence = 0.13f,
                    verticalBeamDivergence = 0.13f,
                }},

                {LidarModel.VelodyneVLP16, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArrayLibrary.VelodyneVLP16,
                    horizontalResolution = 0.2f,
                    laserArrayCycleTime = 0.055296f,
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0,
                    maxRange = 100.0f,
                    horizontalBeamDivergence = 0.171887f, // Manual-based value
                    verticalBeamDivergence = 0.0859435f, // Manual-based value
                }},

                {LidarModel.VelodyneVLP32C, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArrayLibrary.VelodyneVLP32C,
                    horizontalResolution = 0.2f,
                    laserArrayCycleTime = 0.055296f,
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0,
                    maxRange = 200.0f,
                    horizontalBeamDivergence = 0.171887f, // Manual-based value
                    verticalBeamDivergence = 0.0859435f, // Manual-based value
                }},

                {LidarModel.VelodyneVLS128, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArrayLibrary.VelodyneVLS128,
                    horizontalResolution = 0.2f,
                    laserArrayCycleTime = 0.0585688f,
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0,
                    maxRange = 220.0f,
                    horizontalBeamDivergence = 0.171887f, // Manual-based value
                    verticalBeamDivergence = 0.0859435f, // Manual-based value
                }},

                {LidarModel.HesaiPandarQT, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArrayLibrary.HesaiPandarQT,
                    horizontalResolution = 0.6f,
                    laserArrayCycleTime = 0.16667f,
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0.1f,
                    maxRange = 20.0f, // Yes, 20 meters, this is not a typo!
                    horizontalBeamDivergence = 0.13f, // Not specified in manual
                    verticalBeamDivergence = 0.13f, // Not specified in manual
                }},

                {LidarModel.HesaiPandar40P, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArrayLibrary.HesaiPandar40P,
                    horizontalResolution = 0.2f,
                    laserArrayCycleTime = 0.05556f,
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0.3f,
                    // documentation is unclear on max range;
                    // on one hand there is "range capability" = 200m
                    // on the other, in appendix beams have individual ranges assigned
                    // that vary from 130m to 230m
                    // as this template supports single-value range, 200m is chosen
                    maxRange = 200.0f,
                    horizontalBeamDivergence = 0.13f, // Not specified in manual
                    verticalBeamDivergence = 0.13f, // Not specified in manual
                }},


                {LidarModel.OusterOS1_64, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArrayLibrary.OusterOS1_64,
                    horizontalResolution = 360.0f / 1024.0f,
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0.5f,
                    maxRange = 90.0f,
                    horizontalBeamDivergence = 0.13f, // Manual-based value
                    verticalBeamDivergence = 0.13f, // Not specified in manual
                }},

                {LidarModel.HesaiAT128E2X, () => new HesaiAT128LidarConfiguration()
                {
                    laserArray = LaserArrayLibrary.HesaiAT128E2X,
                    horizontalResolution = 0.1f,
                    laserArrayCycleTime = 0.041666f,
                    minHAngle = -60.0f,
                    maxHAngle = 60.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    horizontalBeamDivergence = 0.13f, // Not specified in manual
                    verticalBeamDivergence = 0.13f, // Not specified in manual
                }},

                {LidarModel.HesaiPandarXT32, () => new UniformRangeLidarConfiguration
                {
                    laserArray = LaserArrayLibrary.HesaiPandarXT32,
                    horizontalResolution = 0.18f,
                    laserArrayCycleTime = 0.05f,
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    minRange = 0.05f,
                    maxRange = 80.0f,
                    horizontalBeamDivergence = 0.13f, // Not specified in manual
                    verticalBeamDivergence = 0.13f, // Not specified in manual
                }},

                {LidarModel.HesaiQT128C2X, () => new HesaiQT128C2XLidarConfiguration()
                {
                    laserArray = LaserArrayLibrary.HesaiQT128C2X,
                    horizontalResolution = 0.8f,  // resolution for channels 1-64
                                                  // channels 65-128 will have half of this resolution
                    laserArrayCycleTime = 0.11111f,
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    horizontalBeamDivergence = 0.13f, // Not specified in manual
                    verticalBeamDivergence = 0.13f, // Not specified in manual
                }},

                {LidarModel.HesaiPandar128E4X, () => new HesaiPandar128E4XLidarConfiguration()
                {
                    laserArray = LaserArrayLibrary.HesaiPandar128E4X,
                    horizontalResolution = 0.2f,  // resolution for standard mode
                                                  // if high resolution enabled, high-res channels will have half of this resolution
                    laserArrayCycleTime = 0.055556f,  // time for standard mode
                    minHAngle = 0.0f,
                    maxHAngle = 360.0f,
                    noiseParams = LidarNoiseParams.TypicalNoiseParams,
                    horizontalBeamDivergence = 0.13f, // Not specified in manual
                    verticalBeamDivergence = 0.13f, // Not specified in manual
                    highResolutionModeEnabled = false,
                }},
            };
    }
}
