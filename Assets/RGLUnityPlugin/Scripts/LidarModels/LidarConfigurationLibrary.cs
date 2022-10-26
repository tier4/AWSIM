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

using System.Collections.Generic;

namespace RGLUnityPlugin
{
    public static class LidarConfigurationLibrary
    {
        public static readonly Dictionary<LidarModel, LidarConfiguration> ByModel =
            new Dictionary<LidarModel, LidarConfiguration>
            {
                {LidarModel.RangeMeter, RangeMeter},
                {LidarModel.SickMRS6000, SickMRS6000},
                {LidarModel.VelodyneVLP16, VelodyneVLP16},
                {LidarModel.VelodyneVLP32C, VelodyneVLP32C},
                {LidarModel.VelodyneVLS128, VelodyneVLS128},
                {LidarModel.HesaiPandarQT, HesaiPandarQT},
                {LidarModel.HesaiPandar40P, HesaiPandar40P},
                {LidarModel.OusterOS1_64, OusterOS1_64},
            };

        public static LidarConfiguration RangeMeter => new LidarConfiguration
        {
            laserArray = LaserArray.Uniform(-0.0f, 0.0f, 1),
            horizontalSteps = 1,
            minHAngle = -0,
            maxHAngle = 0,
            maxRange = 40,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
        };

        public static LidarConfiguration SickMRS6000 => new LidarConfiguration
        {
            laserArray = LaserArray.Uniform(-15f, 1.5f, 24),
            horizontalSteps = 924,
            minHAngle = -120,
            maxHAngle = 120,
            maxRange = 40,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
        };

        public static LidarConfiguration VelodyneVLP16 => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.VelodyneVLP16,
            horizontalSteps = 360 * 5, // for 0.2deg resolution
            minHAngle = -180.0f,
            maxHAngle = 180.0f,
            maxRange = 100.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
        };

        public static LidarConfiguration VelodyneVLP32C => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.VelodyneVLP32C,
            horizontalSteps = 360 * 5, // for 0.2deg resolution
            minHAngle = -180.0f,
            maxHAngle = 180.0f,
            maxRange = 200.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
        };

        public static LidarConfiguration VelodyneVLS128 => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.VelodyneVLS128,
            horizontalSteps = 360 * 5, // for 0.2deg resolution
            minHAngle = -180.0f,
            maxHAngle = 180.0f,
            maxRange = 300.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
        };

        public static LidarConfiguration HesaiPandarQT => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.HesaiPandarQT,
            horizontalSteps = 600, // for 0.6deg resolution
            minHAngle = -180.0f,
            maxHAngle = 180.0f,
            maxRange = 20.0f, // Yes, 20 meters, this is not a typo!
            noiseParams = LidarConfiguration.TypicalNoiseParams,
        };

        public static LidarConfiguration HesaiPandar40P => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.HesaiPandar40P,
            horizontalSteps = 360 * 5, // for 0.2deg resolution
            minHAngle = -180.0f,
            maxHAngle = 180.0f,
            // documentation is unclear on max range;
            // on one hand there is "range capability" = 200m
            // on the other, in appendix beams have individual ranges assigned
            // that vary from 130m to 230m
            // as this template supports single-value range, 200m is chosen
            maxRange = 200.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
        };

        public static LidarConfiguration OusterOS1_64 => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.OusterOS1_64,
            horizontalSteps = 1024,
            minHAngle = -180.0f,
            maxHAngle = 180.0f,
            maxRange = 120.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
        };
    }
}