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
                {LidarModel.HesaiAT128E2X, HesaiAT128E2X},
                {LidarModel.HesaiPandarXT32, HesaiPandarXT32},
                {LidarModel.HesaiQT128C2X, HesaiQT128C2X},
                {LidarModel.HesaiPandar128E4X, HesaiPandar128E4X},
            };

        public static LidarConfiguration RangeMeter => new LidarConfiguration
        {
            laserArray = LaserArray.Uniform(-0.0f, 0.0f, 1),
            horizontalResolution = 1.0f,
            minHAngle = -0,
            maxHAngle = 0,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0,
            maxRange = 40,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration SickMRS6000 => new LidarConfiguration
        {
            laserArray = LaserArray.Uniform(-15f, 1.5f, 24),
            horizontalResolution = 240.0f / 924.0f,
            minHAngle = -120,
            maxHAngle = 120,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0,
            maxRange = 40,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration VelodyneVLP16 => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.VelodyneVLP16,
            horizontalResolution = 0.2f,
            laserArrayCycleTime = 0.055296f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0,
            maxRange = 100.0f,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration VelodyneVLP32C => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.VelodyneVLP32C,
            horizontalResolution = 0.2f,
            laserArrayCycleTime = 0.055296f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0,
            maxRange = 200.0f,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration VelodyneVLS128 => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.VelodyneVLS128,
            horizontalResolution = 0.2f,
            laserArrayCycleTime = 0.0585688f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0,
            maxRange = 300.0f,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration HesaiPandarQT => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.HesaiPandarQT,
            horizontalResolution = 0.6f,
            laserArrayCycleTime = 0.16667f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0,
            maxRange = 20.0f, // Yes, 20 meters, this is not a typo!
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration HesaiPandar40P => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.HesaiPandar40P,
            horizontalResolution = 0.2f,
            laserArrayCycleTime = 0.05556f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0,
            // documentation is unclear on max range;
            // on one hand there is "range capability" = 200m
            // on the other, in appendix beams have individual ranges assigned
            // that vary from 130m to 230m
            // as this template supports single-value range, 200m is chosen
            maxRange = 200.0f,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration OusterOS1_64 => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.OusterOS1_64,
            horizontalResolution = 360.0f / 1024.0f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0,
            maxRange = 120.0f,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration HesaiAT128E2X => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.HesaiAT128E2X,
            horizontalResolution = 0.2f,
            laserArrayCycleTime = 0.041666f,
            minHAngle = -60.0f,
            maxHAngle = 60.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.HesaiAT128,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration HesaiPandarXT32 => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.HesaiPandarXT32,
            horizontalResolution = 0.18f,
            laserArrayCycleTime = 0.05f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarEqualRange,
            minRange = 0.05f,
            maxRange = 120.0f,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration HesaiQT128C2X => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.HesaiQT128C2X,
            horizontalResolution = 0.8f,
            laserArrayCycleTime = 0.11111f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.HesaiQT128C2X,
            beamDivergence = 0.13f,
        };

        public static LidarConfiguration HesaiPandar128E4X => new LidarConfiguration
        {
            laserArray = LaserArrayLibrary.HesaiPandar128E4X,
            horizontalResolution = 0.2f,
            laserArrayCycleTime = 0.027778f,
            minHAngle = 0.0f,
            maxHAngle = 360.0f,
            noiseParams = LidarConfiguration.TypicalNoiseParams,
            rayGenerateMethod = LidarConfiguration.RayGenerateMethod.RotatingLidarDifferentLaserRanges,
            beamDivergence = 0.13f,
        };
    }
}
