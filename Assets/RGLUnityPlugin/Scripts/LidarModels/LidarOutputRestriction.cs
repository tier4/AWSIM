// Copyright 2024 Robotec.ai.
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
using UnityEngine;

namespace RGLUnityPlugin
{

    [Serializable]
    public class LidarOutputRestriction
    {
        public LidarOutputRestriction(int cloudSize)
        {
        }
        public LidarOutputRestriction()
        {

        }

        /// <summary>
        /// Allows to quickly enable/disable the restriction.
        /// </summary>
        public bool applyOutputRestriction = false;

        [Tooltip("TODO")]
        [Range(0, 360)] public int startingHorizontalAngle;

        [Tooltip("TODO")]
        [Range(0, 360)] public int endingHorizontalAngle;

        [Tooltip("TODO")]
        [Range(0, 180)] public int startingVerticalAngle;

        [Tooltip("TODO")]
        [Range(0, 180)] public int endingVerticalAngle;

        private sbyte[] raysMask;
        private int cloudSize;

        private void CreateFullMask(int cloudSizeIn)
        {
            cloudSize = cloudSizeIn;

            raysMask = new sbyte[cloudSize];
            for (int i = 0; i < cloudSize; i++)
            {
                raysMask[i] = 1;
            }
        }

        private void CreateRectangularMask(BaseLidarConfiguration configuration)
        {
            int horizontalSteps = configuration.PointCloudSize / 360;
            int verticalSteps = PointCloudSize / horizontalSteps;

        }

        public void ApplyRestriction(RGLNodeSequence rglGraphLidar, string identifier)
        {
            rglGraphLidar.ApplyLidarOutputRestriction(identifier, raysMask);
        }

        public void Update(BaseLidarConfiguration configuration)
        {
            CreateFullMask(configuration.PointCloudSize);
            if(applyOutputRestriction)
            {
                CreateRectangularMask(configuration);
            }
        }
    }
}