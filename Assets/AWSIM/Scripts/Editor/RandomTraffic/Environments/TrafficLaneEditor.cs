using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;

namespace AWSIM.TrafficSimulation
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(TrafficLane))]
    public class TrafficLaneEditor : Editor
    {
        private void OnSceneGUI()
        {
            // var trafficLane = target as TrafficLane;
            // TODO: Handle implementation
        }

        [DrawGizmo(GizmoType.InSelectionHierarchy | GizmoType.Pickable)]
        private static void DrawGizmoNonSelected(TrafficLane trafficLane, GizmoType gizmoType)
        {
            for (int i = 1; i < trafficLane.Waypoints.Length; ++i)
            {
                Gizmos.DrawLine(trafficLane.Waypoints[i - 1], trafficLane.Waypoints[i]);
                Gizmos.DrawCube(trafficLane.Waypoints[i], new Vector3(0.3f, 0.3f, 0.3f));
            }
        }

        [DrawGizmo(GizmoType.Selected)]
        private static void DrawGizmoSelected(TrafficLane trafficLane, GizmoType gizmoType)
        {
            var defaultColor = Gizmos.color;

            Gizmos.color = Color.blue;

            for (int i = 1; i < trafficLane.Waypoints.Length; ++i)
            {
                Gizmos.DrawLine(trafficLane.Waypoints[i - 1], trafficLane.Waypoints[i]);
                Gizmos.DrawSphere(trafficLane.Waypoints[i], 0.3f);
            }

            Gizmos.color = Color.yellow;

            foreach (var lane in trafficLane.RightOfWayLanes)
            {
                for (int i = 1; i < lane.Waypoints.Length; ++i)
                {
                    Gizmos.DrawLine(lane.Waypoints[i - 1], lane.Waypoints[i]);
                    Gizmos.DrawSphere(lane.Waypoints[i], 0.4f);
                }
            }

            Gizmos.color = defaultColor;
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            if (GUILayout.Button("Set RightOfWays"))
            {
                FindAndSetRightOfWays(target as TrafficLane);
            }
        }

        /// <summary>
        /// Set right of ways of <paramref name="lane"/>.<br/>
        /// All lanes that intersect <paramref name="lane"/> besides the following pattern will be set.<br/>
        /// - RIGHT and LEFT against STRAIGHT<br/>
        /// - RIGHT and LEFT against LEFT<br/>
        /// - RIGHT from the right side against RIGHT<br/>
        /// </summary>
        /// <param name="lane">Target <see cref="TrafficLane"/> component</param>
        public static void FindAndSetRightOfWays(TrafficLane lane)
        {
            var lanes = GameObject.FindObjectsOfType<TrafficLane>();
            FindAndSetRightOfWays(lane, lanes);
        }

        /// <summary>
        /// Set right of ways of <paramref name="lane"/>.<br/>
        /// All lanes that intersect <paramref name="lane"/> besides the following pattern will be set.<br/>
        /// - RIGHT and LEFT against STRAIGHT<br/>
        /// - RIGHT and LEFT against LEFT<br/>
        /// - RIGHT from the right side against RIGHT<br/>
        /// </summary>
        /// <param name="lane">Target <see cref="TrafficLane"/> component</param>
        /// <param name="candidateLanes">Candidate lanes to be assigned as right of way</param>
        public static void FindAndSetRightOfWays(TrafficLane lane, IList<TrafficLane> candidateLanes)
        {
            lane.RightOfWayLanes.Clear();

            foreach (var other in candidateLanes)
            {
                if (JudgeRightOfWay(lane, other))
                    lane.RightOfWayLanes.Add(other);
            }
        }

        /// <summary>
        /// Judge if <paramref name="other"/> is right of way of <paramref name="lane"/>.
        /// </summary>
        /// <param name="lane"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        private static bool JudgeRightOfWay(TrafficLane lane, TrafficLane other)
        {
            var otherPosition = other.Waypoints[0];

            // Check lanes that are clearly right of ways or not for reducing computing cost.
            var isTooFar = Vector3.Distance(lane.Waypoints[0], otherPosition) > 300f;
            if (isTooFar)
                return false;

            if (other == lane || AreSamePrevLanes(lane, other) ||
                lane.NextLanes.Contains(other) ||
                other.NextLanes.Contains(lane))
                return false;

            switch (lane.TurnDirection)
            {
                case TrafficLane.TurnDirectionType.STRAIGHT:
                    break;
                case TrafficLane.TurnDirectionType.RIGHT:
                    if (other.TurnDirection == TrafficLane.TurnDirectionType.RIGHT)
                    {
                        //Ignore the right turn lane on the right side, as the left side has priority.
                        var laneForward = lane.Waypoints[1] - lane.Waypoints[0];
                        var otherLanePosition = other.Waypoints[0] - lane.Waypoints[0];
                        var isInRightSide = Vector3.SignedAngle(laneForward, otherLanePosition, Vector3.up) >= 0f;
                        if (isInRightSide)
                            return false;
                    }
                    if (AreSameNextLanes(lane, other))
                        return true;

                    break;
                case TrafficLane.TurnDirectionType.LEFT:
                    if (other.TurnDirection == TrafficLane.TurnDirectionType.RIGHT ||
                        other.TurnDirection == TrafficLane.TurnDirectionType.LEFT)
                        return false;
                    if (AreSameNextLanes(lane, other))
                        return true;
                    break;
            }

            return Intersects(lane, other);
        }

        private static bool AreSamePrevLanes(TrafficLane lane1, TrafficLane lane2)
        {
            return lane1.PrevLanes.Count >= 1 && 
                   lane2.PrevLanes.Count >= 1 && 
                   lane1.PrevLanes[0] == lane2.PrevLanes[0];
        }

        private static bool AreSameNextLanes(TrafficLane lane1, TrafficLane lane2)
        {
            return lane1.NextLanes.Count >= 1 &&
                   lane2.NextLanes.Count >= 1 &&
                   lane1.NextLanes[0] == lane2.NextLanes[0];
        }

        private static bool Intersects(TrafficLane lane1, TrafficLane lane2)
        {
            for (var i1 = 0; i1 < lane1.Waypoints.Length - 1; i1++)
            {
                var p1 = lane1.Waypoints[i1];
                var p2 = lane1.Waypoints[i1 + 1];
                for (var i2 = 0; i2 < lane2.Waypoints.Length - 1; i2++)
                {
                    var q1 = lane2.Waypoints[i2];
                    var q2 = lane2.Waypoints[i2 + 1];
                    if (Intersects(p1, p2, q1, q2))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        private static bool Intersects(Vector3 p1, Vector3 p2, Vector3 q1, Vector3 q2)
        {
            return AreLeftOrder(p1, q1, q2) != AreLeftOrder(p2, q1, q2) && AreLeftOrder(p1, p2, q1) != AreLeftOrder(p1, p2, q2);

            static bool AreLeftOrder(Vector3 p, Vector3 q, Vector3 r)
                => (r.z - p.z) * (q.x - p.x) >= (q.z - p.z) * (r.x - p.x);

        }
    }
}
