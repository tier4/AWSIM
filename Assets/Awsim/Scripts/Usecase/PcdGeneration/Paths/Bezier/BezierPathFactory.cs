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

namespace Awsim.Usecase.PcdGeneration
{
    /// <summary>
    /// A factory for <see cref="BezierPath"/>.
    /// </summary>
    public class BezierPathFactory
    {
        public BezierPath CreateBezierPath(Vector3[] points)
        {
            var bezierPath = new BezierPath(points[0], CalculateHandleVector(points[0], points[0], points[1]));
            for (int i = 1; i < points.Length - 1; ++i)
            {
                bezierPath.Extend(points[i], CalculateHandleVector(points[i], points[i - 1], points[i + 1]));
            }
            bezierPath.Extend(points[points.Length - 1], CalculateHandleVector(points[points.Length - 1], points[points.Length - 2], points[points.Length - 1]), true);
            return bezierPath;
        }

        private Vector3 CalculateHandleVector(Vector3 point, Vector3 prevPoint, Vector3 nextPoint)
        {
            var b = (nextPoint - prevPoint).magnitude;
            var d1 = Vector3.Dot(point - nextPoint, prevPoint - nextPoint) / b;
            var d2 = Vector3.Dot(point - prevPoint, nextPoint - prevPoint) / b;
            var handleLength = d1 == 0f ? d2
                             : d2 == 0f ? d1
                                        : Mathf.Min(d1, d2) / 3f;
            var handleDirection = (nextPoint - prevPoint).normalized;
            return handleDirection * handleLength;
        }

        public static void Convert(PolyLinePath from, out BezierPath to,
                float anchorInterval = 3f, float handleLength = 1f)
        {
            var anchorPosition = 0f;
            to = new BezierPath(from[0], from[handleLength] - from[0], true);
            anchorPosition += anchorInterval;
            while (anchorPosition + anchorInterval + handleLength < from.Length)
            {
                var controlPosition1 = from[anchorPosition - handleLength];
                var controlPosition2 = from[anchorPosition + handleLength];
                var handleVector = (controlPosition2 - controlPosition1) / 2f;
                to.Extend(from[anchorPosition], handleVector, false);
                anchorPosition += anchorInterval;
            }
            to.Extend(from[from.Length], from[from.Length] - from[from.Length - handleLength], true);
        }
    }
}
