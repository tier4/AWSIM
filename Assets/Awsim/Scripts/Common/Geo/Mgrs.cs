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

using System;
using UnityEngine;

namespace Awsim.Common
{
    [Serializable]
    public class Mgrs
    {
        /// <summary>
        /// Reference point of MGRS coordinate system.
        /// </summary>
        public Vector3 Position => _position;

        /// <summary>
        /// MGRS Grid Zone. (e.g. Tokyo is "54SUE")
        /// </summary>
        /// <see href="https://maps.gsi.go.jp/#9/35.499810/138.854828/&base=std&ls=std&disp=1&vs=c1g1j0h0k0l0u1t0z0r0s0m0f1"></see>
        public string GridZone => _gridZone;

        [SerializeField] Vector3 _position;
        [SerializeField] string _gridZone;

        public Mgrs(Vector3 position, string gridZone)
        {
            this._position = position;
            this._gridZone = gridZone;
        }
    }
}