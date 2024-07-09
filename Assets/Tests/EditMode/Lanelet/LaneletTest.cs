using NUnit.Framework;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.TestTools.Utils;
using AWSIM.TrafficSimulation;
using AWSIM;
using AWSIM.Lanelet;
using UnityEditor;

/// <summary>
/// The LaneletTest class manages the testing of the Lanelet Import feature.
/// It handles loading the necessary scenes and executing tests to validate the functionality of the Lanelet Importer.
/// </summary>
public class LaneletTest
{
    string sceneName = "Assets/Tests/EditMode/Lanelet/LaneletTest.unity";
    Scene scene;

    private Transform root = default;
    private Vector3EqualityComparer v3Comparer = new Vector3EqualityComparer(10e-6f);


    // --- TEST LIFE CYCLE ---//

    /// <summary>
    /// Method called by Unity at the start of each test.
    /// </summary>
    [UnitySetUp]
    public IEnumerator Setup()
    {
        yield return LoadScene();

        root = AWSIM.Environment.Instance.transform;
        yield return null;
    }

    /// <summary>
    /// Method to load the scene dedicated to Lanelet Importer tests.
    /// </summary>
    private IEnumerator LoadScene()
    {
        scene = EditorSceneManager.OpenScene(sceneName, OpenSceneMode.Single);
        EditorSceneManager.SetActiveScene(scene);
        Assert.NotNull(scene);

        yield return null;
    }

    /// <summary>
    /// Helper method to import an .osm file via LaneletLoader.
    /// </summary>
    private void ImportLanelet()
    {
        OsmDataContainer osm = AssetDatabase.LoadAssetAtPath<OsmDataContainer>("Assets/Tests/EditMode/Lanelet/lanelet_map_test.osm");
        Assert.NotNull(osm);

        LaneletLoader loader = new LaneletLoader();
        loader.SetWaypointSettings(LaneletLoader.WaypointSettings.Default());
        loader.Load(osm.Data, AWSIM.Environment.Instance.MgrsOffsetPosition, 
            root.gameObject);        
    }

    /// <summary>
    /// Helper method to remove all imported StopLines and TrafficLanes from the Unity scene.
    /// </summary>
    private void DestroyLanelet()
    {
        // remove imported stop lines
        Transform trStops = root.Find("StopLines");
        GameObject.DestroyImmediate(trStops.gameObject);

        // remove imported traffic lanes
        Transform trLanes = root.Find("TrafficLanes");
        GameObject.DestroyImmediate(trLanes.gameObject);

        // remove anything else
        List<Transform> toRemove = new List<Transform>();
        for (int i=0; i<root.childCount; i++)
        {
            toRemove.Add(root.GetChild(i));
        }

        foreach(Transform tr in toRemove)
        {
            GameObject.DestroyImmediate(tr.gameObject);
        }
    }


    // --- TEST ROUTINES --- //

    /// <summary>
    /// Test to validate if the correct number of objects is imported from the .osm file to the Unity scene.
    /// </summary>
    [UnityTest]
    public IEnumerator LaneletImporter_ObjectCount()
    {
        ImportLanelet();

        yield return null;

        Transform trLanes = root.Find("TrafficLanes");
        Assert.NotNull(trLanes);

        Transform trStops = root.Find("StopLines");
        Assert.NotNull(trStops);

        // check the number of traffic lanes
        Assert.AreEqual(trLanes.childCount, 40);

        // check the number of stop line
        Assert.AreEqual(trStops.childCount, 4);

        // clean
        DestroyLanelet();
    }

    /// <summary>
    /// Test to validate if each imported TrafficLane has correctly imported the waypoint array.
    /// </summary>
    [UnityTest]
    public IEnumerator LaneletImporter_TrafficLane_WaypointLenght()
    {
        ImportLanelet();

        yield return null;

        Transform trLanes = root.Find("TrafficLanes");
        Assert.NotNull(trLanes);

        // iterate over all traffic lanes
        for(int i=0; i<trLanes.childCount; i++)
        {
            TrafficLane lane = trLanes.GetChild(i).GetComponent<TrafficLane>();
            Assert.NotNull(lane);
            Assert.NotNull(lane.Waypoints);
            Assert.GreaterOrEqual(lane.Waypoints.Length, 2);
        }

        // clean
        DestroyLanelet();
    }

    /// <summary>
    /// Test to validate if each imported TrafficLane has been placed in the correct position in the Unity scene.
    /// </summary>
    [UnityTest]
    public IEnumerator LaneletImporter_TrafficLane_LanePosition()
    {
        ImportLanelet();

        yield return null;

        Transform trLanes = root.Find("TrafficLanes");
        Assert.NotNull(trLanes);

        // iterate over all traffic lanes
        for(int i=0; i<trLanes.childCount; i++)
        {
            TrafficLane lane = trLanes.GetChild(i).GetComponent<TrafficLane>();
            Assert.NotNull(lane);
            Assert.NotNull(lane.Waypoints);

            // check if traffic lane object has the same position as first point in waypoints array
            Assert.That(lane.transform.position, Is.EqualTo(lane.Waypoints[0]).Using(v3Comparer));
        }

        // clean
        DestroyLanelet();
    }

    /// <summary>
    /// Test to validate if TrafficLanes have been imported correctly and if there are no gaps between 
    /// one traffic lane and the next traffic lane.
    /// </summary>
    [UnityTest]
    public IEnumerator LaneletImporter_NextLanePosition()
    {
        ImportLanelet();

        yield return null;

        Transform trLanes = root.Find("TrafficLanes");
        Assert.NotNull(trLanes);

        // iterate over all traffic lanes
        for(int i=0; i<trLanes.childCount; i++)
        {
            TrafficLane lane = trLanes.GetChild(i).GetComponent<TrafficLane>();
            Assert.NotNull(lane);
            Assert.NotNull(lane.Waypoints);
            Assert.GreaterOrEqual(lane.Waypoints.Length, 2);

            Vector3 endPoint = lane.Waypoints[lane.Waypoints.Length-1];

            // check if last waypoint position of current lane is equal to the first waypoint position of next lane
            foreach(TrafficLane nextLane in lane.NextLanes)
            {
                Assert.NotNull(nextLane.Waypoints);
                Assert.GreaterOrEqual(nextLane.Waypoints.Length, 2);

                Assert.That(endPoint, Is.EqualTo(nextLane.Waypoints[0]).Using(v3Comparer));
            }
        }
    }

    /// <summary>
    /// Test to validate if TrafficLanes have been imported correctly and if there are no gaps between 
    /// one traffic lane and the previous traffic lane.
    /// </summary>
    [UnityTest]
    public IEnumerator LaneletImporter_PreviousLanePosition()
    {
        ImportLanelet();

        yield return null;

        Transform trLanes = root.Find("TrafficLanes");
        Assert.NotNull(trLanes);

        // iterate over all traffic lanes
        for(int i=0; i<trLanes.childCount; i++)
        {
            TrafficLane lane = trLanes.GetChild(i).GetComponent<TrafficLane>();
            Assert.NotNull(lane);
            Assert.NotNull(lane.Waypoints);
            Assert.GreaterOrEqual(lane.Waypoints.Length, 2);

            // check if first waypoint position of current lane is equal to the last waypoint position of previous lane
            foreach(TrafficLane prevLane in lane.PrevLanes)
            {
                Assert.NotNull(prevLane.Waypoints);
                Assert.GreaterOrEqual(prevLane.Waypoints.Length, 2);

                Vector3 endPoint = prevLane.Waypoints[prevLane.Waypoints.Length-1];
                Assert.That(lane.Waypoints[0], Is.EqualTo(endPoint).Using(v3Comparer));
            }
        }
    }

}
