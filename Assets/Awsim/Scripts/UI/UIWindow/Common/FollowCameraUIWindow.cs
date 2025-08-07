// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using Awsim.Common;
using UnityEngine;
using UnityEngine.UI;

namespace Awsim.UI
{
    public class FollowCameraUIWindow : UIWindow
    {
        public FollowCamera FollowCamera { get; set; }
        [SerializeField] Toggle _enableRotateAroundToggle;

        public override void OnStart()
        {
            _enableRotateAroundToggle.isOn = FollowCamera.EnableRotateAround;
            _enableRotateAroundToggle.onValueChanged.AddListener(OnToggleValueChanged);
        }

        void OnToggleValueChanged(bool isOn)
        {
            FollowCamera.EnableRotateAround = isOn;
        }
    }
}