using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

// TODO: think of a better place to put this
// TODO: think of a better name

namespace AWSIM.Lanelet
{
    /// <summary>
    /// A class used for visualizing the left and right bounds of Lanelets in Unity
    /// </summary>
    public class LaneletBoundsVisualizer
    {
        [System.Serializable]
        public struct WaypointSettings
        {
            [SerializeField, Range(0.1f, 2f)]
            [Tooltip("Resolution of resampling. Lower values provide better accuracy at the cost of processing time.")]
            private float resolution;
            [SerializeField, Range(0.3f, 100f)]
            [Tooltip("Minimum length(m) between adjacent points.")]
            private float minDeltaLength;
            [SerializeField, Range(1f, 30f)]
            [Tooltip("Minimum angle(deg) between adjacent edges.")]
            private float minDeltaAngle;

            /// <summary>
            /// Resolution of resampling. Lower values provide better accuracy at the cost of processing time.
            /// </summary>
            public float Resolution => resolution;

            /// <summary>
            /// Minimum length(m) between adjacent points.
            /// </summary>
            public float MinDeltaLength => minDeltaLength;

            /// <summary>
            /// Minimum angle(deg) between adjacent edges.
            /// </summary>
            public float MinDeltaAngle => minDeltaAngle;

            public WaypointSettings(float resolution, float minDeltaLength, float minDeltaAngle)
            {
                this.resolution = resolution;
                this.minDeltaLength = minDeltaLength;
                this.minDeltaAngle = minDeltaAngle;
            }

            public static WaypointSettings Default()
            {
                return new WaypointSettings(1f, 30f, 15f);
            }
        }

        private readonly Dictionary<long, LaneletGameObject> laneletBoundsLeft = new Dictionary<long, LaneletGameObject>();
        private readonly Dictionary<long, LaneletGameObject> laneletBoundsRight = new Dictionary<long, LaneletGameObject>();
        
        private LaneletMap laneletMap;
        private GameObject laneletBoundsHolder;
        private WaypointSettings settings = WaypointSettings.Default();

        public void SetWaypointSettings(WaypointSettings settings)
        {
            this.settings = settings;
        }

        /// <summary>
        /// Load <see cref="OsmData"/> and generate the left and right bounds of lanelet.
        /// </summary>
        /// <param name="osmData">Input OSM data.</param>
        /// <param name="offset">Offset from lanelet origin to Unity scene origin. Usually this is a MGRS coordinate of Unity scene origin.</param>
        /// <param name="holder">GameObject to that generated objects are placed.</param>
        public void Load(OsmData osmData, Vector3 offset, GameObject holder, bool loadRaw)
        {
            laneletMap = new OsmToLaneletMap(offset).Convert(osmData);
            laneletBoundsHolder = new GameObject("LaneletBounds");
            if (holder != null)
            {
                laneletBoundsHolder.transform.parent = holder.transform;
            }

            CreateLaneletBounds(loadRaw);
        }

        private void CreateLaneletBounds(bool loadRaw)
        {
            foreach (var lanelet in laneletMap.Lanelets.Values)
            {
                if (lanelet.Attributes[AttributeKeys.Subtype] != AttributeValues.Road)
                {
                    continue;
                }

                var waypointsLeft = lanelet.LeftBound.Select(p => p.Value).ToArray();
                var waypointsRight = lanelet.RightBound.Select(p => p.Value).ToArray();
                if (!loadRaw)
                {
                    waypointsLeft = lanelet.CalculateLeftLine(settings.Resolution, settings.MinDeltaLength, settings.MinDeltaAngle);
                    waypointsRight = lanelet.CalculateRightLine(settings.Resolution, settings.MinDeltaLength, settings.MinDeltaAngle);
                }
                var laneletBoundLeft = LaneletGameObject.Create(waypointsLeft);
                var laneletBoundRight = LaneletGameObject.Create(waypointsRight);
                laneletBoundLeft.transform.parent = laneletBoundsHolder.transform;
                laneletBoundRight.transform.parent = laneletBoundsHolder.transform;
                laneletBoundsLeft.Add(lanelet.ID, laneletBoundLeft);
                laneletBoundsRight.Add(lanelet.ID, laneletBoundRight);
            }
        }

        [DrawGizmo(GizmoType.InSelectionHierarchy | GizmoType.Pickable)]
        private static void DrawGizmoNonSelected(LaneletGameObject lanelet, GizmoType gizmoType)
        {
            for (int i=1; i < lanelet.Waypoints.Length; i++)
            {
                Gizmos.DrawLine(lanelet.Waypoints[i-1], lanelet.Waypoints[i]);
                Gizmos.DrawSphere(lanelet.Waypoints[i], 0.15f);
            }
        }
    }
}
