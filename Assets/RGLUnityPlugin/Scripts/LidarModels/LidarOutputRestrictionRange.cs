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
    public class LidarOutputRestrictionRange
    {

        [Tooltip("Starting horizontal angle for single rectangular restriction")]
        [Range(0, 360)] public int startingHorizontalAngle;

        [Tooltip("Ending horizontal angle for single rectangular restriction")]
        [Range(0, 360)] public int endingHorizontalAngle;

        [Tooltip("Starting vertical angle for single rectangular restriction")]
        [Range(-90, 90)] public int startingVerticalAngle;

        [Tooltip("Ending vertical angle for single rectangular restriction")]
        [Range(-90, 90)] public int endingVerticalAngle;

    }

}