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
    /// Test Outline:
    ///     - Validate the import process of a .osm file to the Unity scene.
    ///     - Test if correct number of objects are imported from the .osm file.
    /// Test Target:
    ///     - Check if the LaneletLoader script correctly imports .osm data into the Unity scene.
    /// Expected Result:
    ///     - The number of objects imported into the Unity scene matches the expected number from the .osm file.
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
    /// Test Outline:
    ///     - Validate the import process of a .osm file to the Unity scene.
    ///     - Test if each TrafficLane has correctly imported its associated waypoint array.
    /// Test Target:
    ///     - Check if the LaneletLoader script correctly imports .osm data into the Unity scene.
    ///     - Check if each imported TrafficLane contains the valid waypoint array.
    /// Expected Result:
    ///     - Each imported TrafficLane has an imported waypoint array that is not empty and has at least to 2 elements.
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
    /// Test Outline:
    ///     - Validate the import process of a .osm file to the Unity scene.
    ///     - Test the placement of imported TrafficLane objects in the Unity scene.
    /// Test Target:
    ///     - Check if the LaneletLoader script correctly imports .osm data into the Unity scene.
    ///     - Check if each imported TrafficLane is positioned correctly in the Unity scene.
    /// Expected Result:
    ///     - Whether each TrafficLane object is placed at the position equal to the first point in the waypoints array.
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
    /// Test Outline:
    ///     - Validate the import process of a .osm file to the Unity scene.
    ///     - Check for continuity between consecutive TrafficLanes.
    /// Test Target:
    ///     - Check if the LaneletLoader script correctly imports .osm data into the Unity scene.
    ///     - Verify there are no gaps between one TrafficLane and the next.
    /// Expected Result:
    ///     - The value of the last waypoint of one TrafficLane matches the value of the first waypoint of the next TrafficLane.
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
    /// Test Outline:
    ///     - Validate the import process of a .osm file to the Unity scene.
    ///     - Ensure there are no gaps between consecutive TrafficLanes.
    /// Test Target:
    ///     - Check if the LaneletLoader script correctly imports .osm data into the Unity scene.
    ///     - Verify there are no gaps between one TrafficLane and the previous one.
    /// Expected Result:
    ///     - The value of the first waypoint of one TrafficLane matches the value of the last waypoint of the previous TrafficLane.
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
