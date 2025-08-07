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

using UnityEngine;
using UnityEngine.UI;
using Awsim.Entity;

namespace Awsim.UI
{
    public class CameraSensorUIWindow : UIWindow
    {
        public ICameraSensor CameraSensor
        {
            get
            {
                return _cameraSensor;
            }

            set
            {
                _cameraSensor = value;
                _cameraSensor.OnOutput += (CameraSensor.IReadOnlyOutputData outputData) =>
                {
                    _rawImage.texture = outputData.OutputRenderTexture;
                };
            }

        }

        [SerializeField] RawImage _rawImage;
        ICameraSensor _cameraSensor;
    }
}