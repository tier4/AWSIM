using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using ROS2;

namespace AWSIM.TrafficSimulation
{
    [ExecuteInEditMode]
    public class AssignIntersectionTrafficLanes : MonoBehaviour
    {
        [SerializeField, Tooltip("TrafficLanes parent - reference to parent of the GameObjects with the TrafficLane script.")]
        public GameObject trafficLanesObjectsParent;

        [SerializeField, Tooltip("Show details of each intersectionLane assignment.")]
        public bool showDetailsOfEachAssignment;

        [SerializeField, Tooltip("Cleaning up the intersectionLane variable")]
        public bool removeCurrentAssignment;

        [SerializeField, Tooltip("Setup TrafficLane names - with suffix *.<number>")]
        public bool setNamesIncrementally;

        [SerializeField, Tooltip("Assign based on RightOfWayLanes")]
        public bool assignBasedOnRightOfWayLanes;


        private void OnEnable()
        {
            if (CheckReferences())
            {
                if (removeCurrentAssignment)
                    CleanUpIntersectionLanesVariable();

                if (setNamesIncrementally)
                    SetTrafficLaneNames();

                if (assignBasedOnRightOfWayLanes)
                    AssignBasedOnRightOfWayLanes();

                Debug.Log("The AssignIntersectionTrafficLanes script has completed all operations.");
            }
        }

        private bool CheckReferences()
        {
            if (trafficLanesObjectsParent == null)
            {
                Debug.LogError("Reference to parent of the GameObjects with the TrafficLane script is unassigned.");
                return false;

            }
            return true;
        }

        private void CleanUpIntersectionLanesVariable()
        {
            Debug.Log("Start cleaning up the intersectionLane variable.");
            foreach (Transform refTrafficLaneTransform in trafficLanesObjectsParent.transform)
            {
                var refTrafficLane = refTrafficLaneTransform.GetComponent<TrafficLane>();
                if (refTrafficLane != null)
                {
                    refTrafficLane.intersectionLane = false;
                }
                else
                {
                    Debug.LogWarning($"Found GameObject ${refTrafficLaneTransform.name} without TrafficLane script in parent {trafficLanesObjectsParent.name}.");
                }
            }
            Debug.Log("Previous command finished.");
        }

        private void SetTrafficLaneNames()
        {
            Debug.Log("Start setting names incrementally.");
            int trafficLaneNumber = 0;
            foreach (Transform refTrafficLaneTransform in trafficLanesObjectsParent.transform)
            {
                var refTrafficLaneObj = refTrafficLaneTransform.gameObject;
                if (refTrafficLaneObj != null)
                {
                    refTrafficLaneObj.name = $"TrafficLane.{trafficLaneNumber++}";
                }
                else
                {
                    Debug.LogWarning($"Found GameObject ${refTrafficLaneObj.name} without TrafficLane script in parent {trafficLanesObjectsParent.name}.");
                }
            }
            Debug.Log("Previous command finished.");
        }

        private void AssignBasedOnRightOfWayLanes()
        {
            Debug.Log("Start assignment based on RightOfWayLanes.");
            foreach (Transform refTrafficLaneTransform in trafficLanesObjectsParent.transform)
            {
                var refTrafficLane = refTrafficLaneTransform.GetComponent<TrafficLane>();
                if (refTrafficLane == null)
                {
                    Debug.LogWarning($"Found GameObject ${refTrafficLaneTransform.name} without TrafficLane script in parent {trafficLanesObjectsParent.name}.");
                    continue;
                }
                else if (refTrafficLane.intersectionLane == true)
                {
                    // If refTrafficLane is already an intersectionLane, skip this TrafficLane
                    continue;
                }
                else if (refTrafficLane.RightOfWayLanes != null && refTrafficLane.RightOfWayLanes.Count > 0)
                {
                    // If refTrafficLane has RightOfWayLanes, set it as an intersectionLane and skip this TrafficLane
                    refTrafficLane.intersectionLane = true;
                    if (showDetailsOfEachAssignment)
                        Debug.Log($"{refTrafficLaneTransform.name} has been assigned as intersectionLane.");
                    continue;
                }

                // Search for otherTrafficLane that has refTrafficLane in RightOfWayLanes
                foreach (Transform otherTrafficLaneTransform in trafficLanesObjectsParent.transform)
                {
                    var otherTrafficLane = otherTrafficLaneTransform.GetComponent<TrafficLane>();
                    if (otherTrafficLane == null)
                    {
                        Debug.LogWarning($"Found GameObject ${otherTrafficLaneTransform.name} without TrafficLane script in parent {trafficLanesObjectsParent.name}.");
                        continue;
                    }

                    foreach (TrafficLane otherRightOfLane in otherTrafficLane.RightOfWayLanes)
                    {
                        if (otherRightOfLane != null && otherRightOfLane.name == refTrafficLane.name)
                        {
                            refTrafficLane.intersectionLane = true;
                            otherTrafficLane.intersectionLane = true;
                            if (showDetailsOfEachAssignment)
                            {
                                Debug.Log($"{refTrafficLaneTransform.name} has been assigned as intersectionLane.");
                                Debug.Log($"{otherTrafficLaneTransform.name} has been assigned as intersectionLane.");
                            }
                            break;
                        }
                    }
                    // If refTrafficLane is already an intersectionLane -> interrupt further search
                    if (refTrafficLane.intersectionLane == true)
                        break;
                }
            }
            Debug.Log("Previous command finished.");
        }
    }
}


