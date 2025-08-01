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

namespace Awsim.Usecase.PcdGeneration
{
    /// <summary>
    /// Provide information of a anchor point of bezier.
    /// </summary>
    [Serializable]
    public struct BezierAnchor
    {
        /// <summary>
        /// Position of the anchor point.
        /// </summary>
        public Vector3 Position;
        /// <summary>
        /// Position of the control point on the side of start point.
        /// </summary>
        public Vector3 Control1;
        /// <summary>
        /// Position of the control point on the side of end point.
        /// </summary>
        public Vector3 Control2;

        /// <summary>
        /// Initialize <see cref="BezierAnchor"/>
        /// </summary>
        /// <param name="position"></param>
        /// <param name="control1"></param>
        /// <param name="control2"></param>
        public BezierAnchor(Vector3 position, Vector3 control1, Vector3 control2)
        {
            this.Position = position;
            this.Control1 = control1;
            this.Control2 = control2;
        }
    }
}
