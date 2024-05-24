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
        public bool applyRestriction = false;

        /// <summary>
        /// List of rectangular restriction which will be apply to lidar as fault injection.
        /// </summary>
        public List<LidarOutputRestrictionRange> rectangularRestrictionMasks = new List<LidarOutputRestrictionRange>();

        /// <summary>
        /// Allows to change mode from static to blinking restriction mode.
        /// </summary>
        public bool enablePeriodicRestriction = false;

        [Tooltip("The period of the blinking in seconds.")]
        [Min(0.0f)]
        public float restrictionPeriod = 1.0f;

        [Tooltip(@"The duty cycle of the blinking.
                0.5 means that the restriction will be active half of the blinkingPeriod time.
                1.0 means that the restriction will be active all the blinkingPeriod time.
                0.0 means that the restriction will be active none of the blinkingPeriod time.")]
        [Range(0.0f, 1.0f)]
        public float restrictionDutyRate = 0.5f;

        /// <summary>
        /// Allows to change blinking from set to random periods.
        /// </summary>
        public bool enableRestrictionRandomizer = false;

        [Tooltip("The lower bound for random value of period of the blinking in seconds.")]
        [Min(0.0f)]
        public float minRandomPerdiod = 0.0f;

        [Tooltip("The upper bound for random value of period of the blinking in seconds.")]
        [Min(0.0f)]
        public float maxRandomPerdiod = 1.0f;

        private sbyte[] raysMask;
        private sbyte[] fullMask;
        private int cloudSize;

        public IEnumerator coroutine;

        public LidarOutputRestriction() { }

        public void ApplyStaticRestriction(RGLNodeSequence rglGraphLidar, string identifier)
        {
            if (applyRestriction)
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
            if (applyRestriction)
            {
                CreateRectangularMask(configuration);
            }
        }

        public IEnumerator BlinkingRoutine(RGLNodeSequence rglGraphLidar, string identifier)
        {
            if (applyRestriction)
            {
                float restrictionDutyTime = restrictionPeriod * restrictionDutyRate;
                float restrictionTime = restrictionPeriod;

                while (enablePeriodicRestriction)
                {
                    if (enableRestrictionRandomizer)
                    {
                        restrictionTime = UnityEngine.Random.Range(minRandomPerdiod, maxRandomPerdiod);
                        restrictionDutyTime = UnityEngine.Random.Range(0.0f, 1.0f) * restrictionTime;
                    }
                    rglGraphLidar.ApplyLidarOutputRestriction(identifier, raysMask);
                    yield return new WaitForSeconds(restrictionDutyTime);

                    rglGraphLidar.ApplyLidarOutputRestriction(identifier, fullMask);
                    yield return new WaitForSeconds(restrictionTime - restrictionDutyTime);
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

                    if (horizontalAngle < 0.0f) {
                        horizontalAngle += 360.0f;
                    } else if (horizontalAngle > 360.0f) {
                        horizontalAngle -= 360.0f;
                    }

                    raysMask[idx] = 1;

                    foreach (var mask in rectangularRestrictionMasks)
                    {
                        if ((horizontalAngle >= mask.startingHorizontalAngle && horizontalAngle <= mask.endingHorizontalAngle) &&
                         (verticalAngle >= mask.startingVerticalAngle && verticalAngle <= mask.endingVerticalAngle))
                        {
                            raysMask[idx] = 0;
                            break;
                        }
                    }
                }
            }
        }
    }
}