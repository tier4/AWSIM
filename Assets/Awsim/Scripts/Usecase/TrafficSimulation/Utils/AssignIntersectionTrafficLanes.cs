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

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// To make the yielding rules work properly, it is necessary to catagorize the TrafficLanes. 
    /// The ones that belong to an intersection have the IntersectionLane variable set to true.
    /// To automate the assignment of the corresponding IntersectionLane to each TrafficLane, this script can be used. 
    /// </summary>
    [Obsolete("This feature has been transported to `LaneletLoader`", false)]
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


        void OnEnable()
        {
            // checks if the reference to the parent of objects containing TrafficLanes has been assigned
            if (CheckReferences())
            {
                // allows to remove current intersectionLane assignments in all children 
                if (removeCurrentAssignment)
                    CleanUpIntersectionLanesVariable();

                // allows to add incremented prefix ".<number" to names in all children, to have individual names
                if (setNamesIncrementally)
                    SetTrafficLaneNames();

                // allows to assign an IntersectionLane to a specific TrafficLane based on the RightOfWayLanes configuration of the others
                if (assignBasedOnRightOfWayLanes)
                    AssignBasedOnRightOfWayLanes();

                Debug.Log("The AssignIntersectionTrafficLanes script has completed all operations.");
            }
        }

        bool CheckReferences()
        {
            if (trafficLanesObjectsParent == null)
            {
                Debug.LogError("Reference to parent of the GameObjects with the TrafficLane script is unassigned.");
                return false;

            }
            return true;
        }

        void CleanUpIntersectionLanesVariable()
        {
            Debug.Log("Start cleaning up the intersectionLane variable.");
            foreach (Transform refTrafficLaneTransform in trafficLanesObjectsParent.transform)
            {
                var refTrafficLane = refTrafficLaneTransform.GetComponent<TrafficLane>();
                if (refTrafficLane != null)
                {
                    refTrafficLane._intersectionLane = false;
                }
                else
                {
                    Debug.LogWarning($"Found GameObject ${refTrafficLaneTransform.name} without TrafficLane script in parent {trafficLanesObjectsParent.name}.");
                }
            }
            Debug.Log("Previous command finished.");
        }

        void SetTrafficLaneNames()
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


        void AssignBasedOnRightOfWayLanes()
        {
            Debug.Log("Start assignment based on RightOfWayLanes.");
            // iterate through all the children of trafficLanesObjectsParent
            foreach (Transform refTrafficLaneTransform in trafficLanesObjectsParent.transform)
            {
                var refTrafficLane = refTrafficLaneTransform.GetComponent<TrafficLane>();
                // check if the child has an TrafficLane script
                if (refTrafficLane == null)
                {
                    Debug.LogWarning($"Found GameObject ${refTrafficLaneTransform.name} without TrafficLane script in parent {trafficLanesObjectsParent.name}.");
                    continue;
                }
                // If refTrafficLane is already an intersectionLane, skip this TrafficLane
                else if (refTrafficLane._intersectionLane == true)
                {
                    continue;
                }
                // If refTrafficLane has RightOfWayLanes, set it as an intersectionLane and skip this TrafficLane
                else if (refTrafficLane.RightOfWayLanes != null && refTrafficLane.RightOfWayLanes.Count > 0)
                {
                    refTrafficLane._intersectionLane = true;
                    if (showDetailsOfEachAssignment)
                        Debug.Log($"{refTrafficLaneTransform.name} has been assigned as intersectionLane.");
                    continue;
                }
                // Else search for otherTrafficLane that has refTrafficLane in RightOfWayLanes, if found, assign intersectionLane as true and break
                else
                {
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
                                refTrafficLane._intersectionLane = true;
                                otherTrafficLane._intersectionLane = true;
                                if (showDetailsOfEachAssignment)
                                {
                                    Debug.Log($"{refTrafficLaneTransform.name} has been assigned as intersectionLane.");
                                    Debug.Log($"{otherTrafficLaneTransform.name} has been assigned as intersectionLane.");
                                }
                                break;
                            }
                        }
                        // If refTrafficLane is already an intersectionLane -> interrupt further search
                        if (refTrafficLane._intersectionLane == true)
                            break;
                    }
                }
            }
            Debug.Log("Previous command finished.");
        }
    }
}


