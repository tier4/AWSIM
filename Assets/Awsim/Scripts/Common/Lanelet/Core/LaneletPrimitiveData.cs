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

using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Awsim.Common
{
    /// <summary>
    /// Common data class for all lanelet primitives.
    /// </summary>
    public class LaneletPrimitiveData
    {
        /// <summary>
        /// Get or set the unique Id of the primitive.
        /// </summary>
        public long Id { get; set; }

        /// <summary>
        /// Get the attributes of the primitive.
        /// </summary>
        public LaneletAttribute Attributes { get; private set; }

        /// <summary>
        /// Initialize <see cref="LaneletPrimitiveData"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        public LaneletPrimitiveData(long id, LaneletAttribute attributes)
        {
            this.Id = id;
            this.Attributes = attributes;
        }
    }

    /// <summary>
    /// Represent point data of lanelet.
    /// </summary>
    public class LaneletPointData : LaneletPrimitiveData
    {
        /// <summary>
        /// Get the position of the point.
        /// </summary>
        public Vector3 Value { get; private set; }

        /// <summary>
        /// Initialize <see cref="LaneletPointData"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="value"></param>
        public LaneletPointData(long id, LaneletAttribute attributes, Vector3 value) : base(id, attributes)
        {
            this.Value = value;
        }

        public static implicit operator Vector3(LaneletPointData p) => p.Value;
    }

    /// <summary>
    /// Represent line data of lanelet.
    /// </summary>
    public class LaneletLineStringData : LaneletPrimitiveData, IEnumerable<LaneletPointData>
    {
        private float length = 0f;

        public LaneletPointData this[int index]
        {
            get => Points[index];
            set => Points[index] = value;
        }

        public IEnumerator<LaneletPointData> GetEnumerator()
        {
            for (var i = 0; i < Points.Length; i++)
            {
                yield return this[i];
            }
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        /// <summary>
        /// Get the array of the line points.
        /// </summary>
        public LaneletPointData[] Points { get; private set; }

        /// <summary>
        /// Get the length of the string of lines.
        /// </summary>
        public float Length
        {
            get
            {
                if (this.length == 0f)
                {
                    this.length = CalculateLength();
                }
                return this.length;
            }
        }

        /// <summary>
        /// Initialize <see cref="LaneletLineStringData"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="points"></param>
        public LaneletLineStringData(long id, LaneletAttribute attributes, LaneletPointData[] points) : base(id, attributes)
        {
            this.Points = points;
        }

        private float CalculateLength()
        {
            var length = 0f;
            for (int i = 0; i < this.Points.Length - 1; ++i)
            {
                length += Vector3.Distance(this.Points[i], this.Points[i + 1]);
            }
            return length;
        }
    }

    /// <summary>
    /// Represent lanelet data of lanelet.
    /// </summary>
    public class LaneletData : LaneletPrimitiveData
    {
        /// <summary>
        /// Get the left bound of lanelet.
        /// </summary>
        public LaneletLineStringData LeftBound { get; private set; }

        /// <summary>
        /// Get the right bound of lanelet.
        /// </summary>
        public LaneletLineStringData RightBound { get; private set; }

        public LaneletRegulatoryElement[] RegulatoryElements { get; set; }

        /// <summary>
        /// Initialize <see cref="LaneletData"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="leftBound"></param>
        /// <param name="rightBound"></param>
        public LaneletData(long id, LaneletAttribute attributes, LaneletLineStringData leftBound, LaneletLineStringData rightBound) : base(id, attributes)
        {
            this.LeftBound = leftBound;
            this.RightBound = rightBound;
        }

        /// <summary>
        /// Calculate centerline of lanelet.
        /// </summary>
        /// <param name="resolution">Resolution of resampling. Lower values provide better accuracy at the cost of processing time.</param>
        /// <param name="minDeltaLength">Minimum length between adjacent points.</param>
        /// <param name="minDeltaAngle">Minimum angle between adjacent edges.</param>
        /// <returns>Center points.</returns>
        public Vector3[] CalculateCenterline(float resolution = 1f, float minDeltaLength = 30f, float minDeltaAngle = 15f)
        {
            var leftPoints = this.LeftBound.Select(p => p.Value).ToArray();
            var rightPoints = this.RightBound.Select(p => p.Value).ToArray();
            var centerPoints = LaneletGeometryUtility.CalculateCenterline(leftPoints, rightPoints, resolution, minDeltaLength, minDeltaAngle);
            return centerPoints;
        }

        public Vector3[] CalculateLeftLine(float resolution = 1f, float minDeltaLength = 30f, float minDeltaAngle = 15f)
        {
            var leftPoints = this.LeftBound.Select(p => p.Value).ToArray();
            var reducedLeftPoints = LaneletGeometryUtility.CalculateSideLine(leftPoints, resolution, minDeltaLength, minDeltaAngle);
            return reducedLeftPoints;
        }

        public Vector3[] CalculateRightLine(float resolution = 1f, float minDeltaLength = 30f, float minDeltaAngle = 15f)
        {
            var rightPoints = this.RightBound.Select(p => p.Value).ToArray();
            var reducedRightPoints = LaneletGeometryUtility.CalculateSideLine(rightPoints, resolution, minDeltaLength, minDeltaAngle);
            return reducedRightPoints;
        }

        public bool IsNextTo(LaneletData other)
        {
            return this.LeftBound.Any() && this.RightBound.Any() &&
                   other.LeftBound.Any() && other.RightBound.Any() &&
                   this.LeftBound.First() == other.LeftBound.Last() &&
                   this.RightBound.First() == other.RightBound.Last();
        }
    }

    public enum LaneletRegulatoryElementType
    {
        TrafficSign,
        TrafficLight,
        RoadMarking,
        RightOfWay
    }

    /// <summary>
    /// Represent regulatory element data of lanelet.
    /// </summary>
    public class LaneletRegulatoryElement : LaneletPrimitiveData
    {
        /// <summary>
        /// Get the type of the regulatory element.
        /// </summary>
        public LaneletRegulatoryElementType Type { get; set; }

        /// <summary>
        /// Get the line from which a restrictions becomes valid.
        /// </summary>
        public LaneletLineStringData[] RefLines { get; private set; }

        /// <summary>
        /// Get the primitives that are the very origin of the restriction. Traffic lights/signs, etc.
        /// </summary>
        public LaneletLineStringData[] Refers { get; private set; }

        /// <summary>
        /// Get the primitives of light bulbs.
        /// </summary>
        public LaneletLineStringData[] LightBulbs { get; private set; }

        /// <summary>
        /// Get the lanelets that have to yield.
        /// </summary>
        public LaneletData[] Yields { get; private set; }

        /// <summary>
        /// Get the lanelets that have the right of way over the yielding ones.
        /// </summary>
        public LaneletData[] RightOfWays { get; private set; }

        public LaneletRegulatoryElement(long id, LaneletAttribute attributes) : base(id, attributes)
        {
        }

        /// <summary>
        /// Create <see cref="LaneletRegulatoryElement"/> instance that is a type of <see cref="LaneletRegulatoryElementType.RightOfWay"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="yields">The yielding lanelets.</param>
        /// <param name="rightOfWays">The lanelets that have the right of way.</param>
        /// <returns></returns>
        public static LaneletRegulatoryElement CreateRightOfWay(
            long id, LaneletAttribute attributes,
            LaneletData[] yields, LaneletData[] rightOfWays)
            => new LaneletRegulatoryElement(id, attributes)
            {
                Type = LaneletRegulatoryElementType.RightOfWay,
                Yields = yields,
                RightOfWays = rightOfWays
            };

        /// <summary>
        /// Create <see cref="LaneletRegulatoryElement"/> instance that is a type of <see cref="LaneletRegulatoryElementType.TrafficSign"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="refLines">The primitive of the stop line.</param>
        /// <param name="refers">The primitive of the traffic sign.</param>
        /// <returns></returns>
        public static LaneletRegulatoryElement CreateTrafficSign(
            long id, LaneletAttribute attributes,
            LaneletLineStringData[] refLines, LaneletLineStringData[] refers)
            => new LaneletRegulatoryElement(id, attributes)
            {
                Type = LaneletRegulatoryElementType.TrafficSign,
                RefLines = refLines,
                Refers = refers
            };

        public static LaneletRegulatoryElement CreateTrafficLight(
            long id, LaneletAttribute attributes,
            LaneletLineStringData[] refLines, LaneletLineStringData[] refers,
            LaneletLineStringData[] lightBulbs)
            => new LaneletRegulatoryElement(id, attributes)
            {
                Type = LaneletRegulatoryElementType.TrafficLight,
                RefLines = refLines,
                Refers = refers,
                LightBulbs = lightBulbs
            };
    }
}
