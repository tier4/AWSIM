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
using System.Collections;
using System.Collections.Generic;

namespace RGLUnityPlugin
{

    [Serializable]
    public class LidarOutputRestriction
    {
        /// <summary>
        /// Allows to quickly enable/disable the restriction.
        /// </summary>
        public bool applyOutputRestriction = false;

        /// <summary>
        /// Allows to change mode from static to blinking restriction mode.
        /// </summary>
        public bool applyOutputBlinking = false;

        /// <summary>
        /// Allows to change blinking from set to random periods.
        /// </summary>
        public bool outputBlinkingRandomizer = false;

        [Tooltip("The period of the blinking in seconds.")]
        [Min(0.0f)]
        public float blinkingPeriod = 1.0f;

        [Tooltip(@"The duty cycle of the blinking.
                0.5 means that the restriction will be active half of the blinkingPeriod time.
                1.0 means that the restriction will be active all the blinkingPeriod time.
                0.0 means that the restriction will be active none of the blinkingPeriod time.")]
        [Range(0.0f, 1.0f)]
        public float dutyRate = 0.5f;

        public List<LidarOutputRestrictionRange> ranges = new List<LidarOutputRestrictionRange>();

        private sbyte[] raysMask;
        private sbyte[] fullMask;
        private int cloudSize;

        public LidarOutputRestriction() { }

        public void ApplyStaticRestriction(RGLNodeSequence rglGraphLidar, string identifier)
        {
            if (applyOutputRestriction)
            {
                rglGraphLidar.ApplyLidarOutputRestriction(identifier, raysMask);
            }
            else
            {
                rglGraphLidar.ApplyLidarOutputRestriction(identifier, fullMask);
            }
        }

        public void Update(BaseLidarConfiguration configuration)
        {
            CreateFullMask(configuration.PointCloudSize);
            if (applyOutputRestriction)
            {
                CreateRectangularMask(configuration);
            }
        }

        public IEnumerator BlinkingRoutine(RGLNodeSequence rglGraphLidar, string identifier)
        {
            if (applyOutputRestriction)
            {
                float dutyTime = blinkingPeriod * dutyRate;
                float blinkingTime = blinkingPeriod;

                while (applyOutputBlinking)
                {
                    if (outputBlinkingRandomizer)
                    {
                        blinkingTime = UnityEngine.Random.Range(0.0f, 2.0f);
                        dutyTime = UnityEngine.Random.Range(0.0f, blinkingTime);
                    }
                    rglGraphLidar.ApplyLidarOutputRestriction(identifier, raysMask);
                    yield return new WaitForSeconds(dutyTime);

                    rglGraphLidar.ApplyLidarOutputRestriction(identifier, fullMask);
                    yield return new WaitForSeconds(blinkingTime - dutyTime);
                }
            }
        }

        private void CreateFullMask(int cloudSizeIn)
        {
            cloudSize = cloudSizeIn;

            fullMask = new sbyte[cloudSize];
            for (int i = 0; i < cloudSize; i++)
            {
                fullMask[i] = 1;
            }
        }

        private void CreateRectangularMask(BaseLidarConfiguration configuration)
        {
            int horizontalSteps = configuration.HorizontalSteps;
            int verticalSteps = configuration.laserArray.lasers.Length;

            raysMask = new sbyte[configuration.PointCloudSize];

            for (int hStep = 0; hStep < horizontalSteps; hStep++)
            {
                for (int laserId = 0; laserId < verticalSteps; laserId++)
                {
                    int idx = laserId + hStep * verticalSteps;

                    float verticalAngle = configuration.laserArray.lasers[laserId].verticalAngularOffsetDeg;

                    float azimuth = configuration.minHAngle + hStep * configuration.horizontalResolution;
                    float horizontalAngle = azimuth + configuration.laserArray.lasers[laserId].horizontalAngularOffsetDeg;

                    raysMask[idx] = 1;

                    foreach (var range in ranges)
                    {
                        if ((horizontalAngle > range.startingHorizontalAngle && horizontalAngle < range.endingHorizontalAngle) &&
                         (verticalAngle > range.startingVerticalAngle && verticalAngle < range.endingVerticalAngle))
                        {
                            raysMask[idx] = 0;
                        }
                    }
                }
            }
        }
    }
}