using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM.Lanelet
{
    /// <summary>
    /// Provide utility static functions for a geometry calculation.
    /// </summary>
    public static class GeometryUtility
    {
        /// <summary>
        /// Calculate length of a polyline that connects all the points.
        /// </summary>
        /// <param name="points"></param>
        /// <returns>Calculated length</returns>
        public static float Length(Vector3[] points)
        {
            var length = 0f;
            for (var i = 0; i < points.Length - 1; i++)
            {
                length += Vector3.Distance(points[i], points[i + 1]);
            }

            return length;
        }

        /// <summary>
        /// Resample <paramref name="points"/> into <paramref name="resampledPointCount"/> points.
        /// <paramref name="points"/> are linealy interpolated before resampling.
        /// </summary>
        /// <param name="points"></param>
        /// <param name="resampledPointCount"></param>
        /// <returns>Array of resampled points</returns>
        public static Vector3[] ResamplePoints(Vector3[] points, int resampledPointCount)
        {
            var resampledPoints = new List<Vector3>();
            resampledPoints.Add(points[0]);

            var length = Length(points);
            var lengths = new List<float>();
            for (var i = 0; i < points.Length - 1; i++)
            {
                lengths.Add(Vector3.Distance(points[i], points[i + 1]));
            }
            var interval = length / (resampledPointCount - 1);
            var inputPointIndex = 1;
            for (var i = 0; i < resampledPointCount - 2; i++)
            {
                if (inputPointIndex == points.Length)
                {
                    break;
                }

                // Find an edge that is a certain distance away from the last sampled point.
                var candidatePoint = resampledPoints.Last();
                float distanceToNextPoint;
                float distanceToNextSamplePoint = interval;
                while (true)
                {
                    distanceToNextPoint = Vector3.Distance(candidatePoint, points[inputPointIndex]);

                    // Next sample point is in the current edge(between points[inputPointIndex - 1] and points[inputPointIndex])
                    if (distanceToNextSamplePoint <= distanceToNextPoint)
                        break;

                    // Go to next edge.
                    distanceToNextSamplePoint -= distanceToNextPoint;
                    candidatePoint = points[inputPointIndex];
                    inputPointIndex++;
                }

                // Interpolate to find a sample point.
                var t = distanceToNextSamplePoint / distanceToNextPoint;
                candidatePoint = candidatePoint * (1 - t) + points[inputPointIndex] * t;
                resampledPoints.Add(candidatePoint);
            }
            resampledPoints.Add(points.Last());
            return resampledPoints.ToArray();
        }

        /// <summary>
        /// Reduce the number of points of the polyline according to the parameters.
        /// </summary>
        /// <param name="minDeltaLength">Minimum length between adjacent points.</param>
        /// <param name="minDeltaAngle">Minimum angle between adjacent edges.</param>
        public static Vector3[] ReducePolyLinePoints(Vector3[] points, float minDeltaLength, float minDeltaAngle)
        {
            if (points == null || points.Length == 0)
                return new Vector3[] { };

            if (points.Length == 1)
                return new[] { points[0] };

            var resampledPoints = new List<Vector3>();
            var lastVertex = points[0];
            var lastEdge = points[1] - points[0];
            resampledPoints.Add(lastVertex);
            for (var i = 1; i < points.Length - 1; ++i)
            {
                var candidateEdge = points[i] - points[i - 1];
                var deltaLength = Vector3.Distance(lastVertex, points[i]);
                var deltaAngle = Vector3.Angle(candidateEdge, lastEdge);

                // Skip a redundant point
                if (deltaLength < minDeltaLength && 
                    deltaAngle < minDeltaAngle)
                    continue;
                
                lastVertex = points[i];
                lastEdge = candidateEdge;
                resampledPoints.Add(lastVertex);
            }

            resampledPoints.Add(points.Last());
            return resampledPoints.ToArray();
        }

        /// <summary>
        /// Calculate centerline of <paramref name="leftLine"/> and <paramref name="rightLine"/>.
        /// </summary>
        /// <param name="leftLine"></param>
        /// <param name="rightLine"></param>
        /// <param name="resolution">Resolution of resampling. Lower values provide better accuracy at the cost of processing time.</param>
        /// <param name="minDeltaLength">Minimum length between adjacent points.</param>
        /// <param name="minDeltaAngle">Minimum angle between adjacent edges.</param>
        /// <returns></returns>
        public static Vector3[] CalculateCenterline(Vector3[] leftLine, Vector3[] rightLine, float resolution = 1f, float minDeltaLength = 30f, float minDeltaAngle = 15f)
        {
            var leftLength = Length(leftLine);
            var rightLength = Length(rightLine);
            var longerLength = Mathf.Max(leftLength, rightLength);
            var pointCount = Mathf.Max(Mathf.CeilToInt(longerLength / resolution) + 1, 2);

            var resampledLeftPoints = ResamplePoints(leftLine, pointCount);
            var resampledRightPoints = ResamplePoints(rightLine, pointCount);
            var centerPoints = new Vector3[pointCount];
            for (int i = 0; i < pointCount; ++i)
            {
                centerPoints[i] = (resampledLeftPoints[i] + resampledRightPoints[i]) / 2;
            }

            centerPoints = ReducePolyLinePoints(centerPoints, minDeltaLength, minDeltaAngle);
            return centerPoints;
        }

        public static Vector3[] CalculateSideLine(Vector3[] sideLine, float resolution = 1f, float minDeltaLength = 30f, float minDeltaAngle = 15f)
        {
            var lineLength = Length(sideLine);
            var pointCount = Mathf.Max(Mathf.CeilToInt(lineLength / resolution) + 1, 2);
            var resampledPoints = ResamplePoints(sideLine, pointCount);
            resampledPoints = ReducePolyLinePoints(resampledPoints, minDeltaLength, minDeltaAngle);
            return resampledPoints;
        }

        public static float Distance2D(Vector3 p1, Vector3 p2)
        {
            var vec = p1 - p2;
            vec.y = 0f;
            return vec.magnitude;
        }
    }
}
