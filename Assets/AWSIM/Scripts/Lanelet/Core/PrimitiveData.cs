using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

namespace AWSIM.Lanelet
{
    /// <summary>
    /// Common data class for all lanelet primitives.
    /// </summary>
    public class PrimitiveData
    {
        /// <summary>
        /// Get or set the unique ID of the primitive.
        /// </summary>
        public long ID { get; set; }

        /// <summary>
        /// Get the attributes of the primitive.
        /// </summary>
        public Attributes Attributes { get; private set; }

        /// <summary>
        /// Initialize <see cref="PrimitiveData"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        public PrimitiveData(long id, Attributes attributes)
        {
            this.ID = id;
            this.Attributes = attributes;
        }
    }

    /// <summary>
    /// Represent point data of lanelet.
    /// </summary>
    public class PointData : PrimitiveData
    {
        /// <summary>
        /// Get the position of the point.
        /// </summary>
        public Vector3 Value { get; private set; }

        /// <summary>
        /// Initialize <see cref="PointData"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="value"></param>
        public PointData(long id, Attributes attributes, Vector3 value) : base(id, attributes)
        {
            this.Value = value;
        }

        public static implicit operator Vector3(PointData p) => p.Value;
    }

    /// <summary>
    /// Represent line data of lanelet.
    /// </summary>
    public class LineStringData : PrimitiveData, IEnumerable<PointData>
    {
        private float length = 0f;

        public PointData this[int index]
        {
            get => Points[index];
            set => Points[index] = value;
        }

        public IEnumerator<PointData> GetEnumerator()
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
        public PointData[] Points { get; private set; }

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
        /// Initialize <see cref="LineStringData"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="points"></param>
        public LineStringData(long id, Attributes attributes, PointData[] points) : base(id, attributes)
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
    public class LaneletData : PrimitiveData
    {
        /// <summary>
        /// Get the left bound of lanelet.
        /// </summary>
        public LineStringData LeftBound { get; private set; }

        /// <summary>
        /// Get the right bound of lanelet.
        /// </summary>
        public LineStringData RightBound { get; private set; }

        public RegulatoryElement[] RegulatoryElements { get; set; }

        /// <summary>
        /// Initialize <see cref="LaneletData"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="leftBound"></param>
        /// <param name="rightBound"></param>
        public LaneletData(long id, Attributes attributes, LineStringData leftBound, LineStringData rightBound) : base(id, attributes)
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
            var centerPoints = GeometryUtility.CalculateCenterline(leftPoints, rightPoints, resolution, minDeltaLength, minDeltaAngle);
            return centerPoints;
        }

        public Vector3[] CalculateLeftLine(float resolution = 1f, float minDeltaLength = 30f, float minDeltaAngle = 15f)
        {
            var leftPoints = this.LeftBound.Select(p => p.Value).ToArray();
            var reducedLeftPoints = GeometryUtility.CalculateSideLine(leftPoints, resolution, minDeltaLength, minDeltaAngle);
            return reducedLeftPoints;
        }

        public Vector3[] CalculateRightLine(float resolution = 1f, float minDeltaLength = 30f, float minDeltaAngle = 15f)
        {
            var rightPoints = this.RightBound.Select(p => p.Value).ToArray();
            var reducedRightPoints = GeometryUtility.CalculateSideLine(rightPoints, resolution, minDeltaLength, minDeltaAngle);
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

    public enum RegulatoryElementType
    {
        TRAFFIC_SIGN,
        TRAFFIC_LIGHT,
        ROAD_MARKING,
        RIGHT_OF_WAY
    }

    /// <summary>
    /// Represent regulatory element data of lanelet.
    /// </summary>
    public class RegulatoryElement : PrimitiveData
    {
        /// <summary>
        /// Get the type of the regulatory element.
        /// </summary>
        public RegulatoryElementType Type { get; set; }

        /// <summary>
        /// Get the line from which a restrictions becomes valid.
        /// </summary>
        public LineStringData[] RefLines { get; private set; }

        /// <summary>
        /// Get the primitives that are the very origin of the restriction. Traffic lights/signs, etc.
        /// </summary>
        public LineStringData[] Refers { get; private set; }

        /// <summary>
        /// Get the primitives of light bulbs.
        /// </summary>
        public LineStringData[] LightBulbs { get; private set; }

        /// <summary>
        /// Get the lanelets that have to yield.
        /// </summary>
        public LaneletData[] Yields { get; private set; }

        /// <summary>
        /// Get the lanelets that have the right of way over the yielding ones.
        /// </summary>
        public LaneletData[] RightOfWays { get; private set; }

        public RegulatoryElement(long id, Attributes attributes) : base(id, attributes)
        {
        }

        /// <summary>
        /// Create <see cref="RegulatoryElement"/> instance that is a type of <see cref="RegulatoryElementType.RIGHT_OF_WAY"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="yields">The yielding lanelets.</param>
        /// <param name="rightOfWays">The lanelets that have the right of way.</param>
        /// <returns></returns>
        public static RegulatoryElement CreateRightOfWay(
            long id, Attributes attributes,
            LaneletData[] yields, LaneletData[] rightOfWays)
            => new RegulatoryElement(id, attributes)
            {
                Type = RegulatoryElementType.RIGHT_OF_WAY,
                Yields = yields,
                RightOfWays = rightOfWays
            };

        /// <summary>
        /// Create <see cref="RegulatoryElement"/> instance that is a type of <see cref="RegulatoryElementType.TRAFFIC_SIGN"/>.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="attributes"></param>
        /// <param name="refLines">The primitive of the stop line.</param>
        /// <param name="refers">The primitive of the traffic sign.</param>
        /// <returns></returns>
        public static RegulatoryElement CreateTrafficSign(
            long id, Attributes attributes,
            LineStringData[] refLines, LineStringData[] refers)
            => new RegulatoryElement(id, attributes)
            {
                Type = RegulatoryElementType.TRAFFIC_SIGN,
                RefLines = refLines,
                Refers = refers
            };

        public static RegulatoryElement CreateTrafficLight(
            long id, Attributes attributes,
            LineStringData[] refLines, LineStringData[] refers,
            LineStringData[] lightBulbs)
            => new RegulatoryElement(id, attributes)
            {
                Type = RegulatoryElementType.TRAFFIC_LIGHT,
                RefLines = refLines,
                Refers = refers,
                LightBulbs = lightBulbs
            };
    }
}
