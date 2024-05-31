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


public class LaneletTest
{
    string sceneName = "Assets/Tests/EditMode/Lanelet/LaneletTest.unity";
    Scene scene;
    AsyncOperation aOp;


    private Transform root = default;
    //private LaneletTestUtils laneletTestUtils = default;

    private LaneletLoader.WaypointSettings waypointSettings = LaneletLoader.WaypointSettings.Default();

    [UnitySetUp]
    public IEnumerator Setup()
    {
        yield return LoadSceneAsync();

        root = AWSIM.Environment.Instance.transform;
        yield return null;
        
    }

    private IEnumerator LoadSceneAsync()
    {
        scene = EditorSceneManager.OpenScene(sceneName, OpenSceneMode.Single);

        // Scene async operation
        //aOp = EditorSceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);

        //yield return new WaitUntil(() => aOp.isDone);
        //scene = EditorSceneManager.GetSceneByName(sceneName);
        EditorSceneManager.SetActiveScene(scene);
        Assert.NotNull(scene);

        yield return null;
    }

    [UnityTest]
    public IEnumerator LaneletImporter_Main()
    {
        //string[] guids2 = AssetDatabase.FindAssets("lanelet_map_test", new[] {"Assets/Tests/EditMode/Lanelet"});
        OsmDataContainer osm = AssetDatabase.LoadAssetAtPath<OsmDataContainer>("Assets/Tests/EditMode/Lanelet/lanelet_map_test.osm");
        Debug.Log(osm);

        LaneletLoader loader = new LaneletLoader();
        loader.SetWaypointSettings(LaneletLoader.WaypointSettings.Default());
        loader.Load(osm.Data, AWSIM.Environment.Instance.MgrsOffsetPosition, 
            root.gameObject);

        yield return null;

        Transform trLanes = root.Find("TrafficLanes");
        Assert.NotNull(trLanes);

        Transform trStops = root.Find("StopLines");
        Assert.NotNull(trStops);

        Assert.AreEqual(trLanes.childCount, 40);

        for(int i=0; i<trLanes.childCount; i++)
        {
            TrafficLane lane = trLanes.GetChild(i).GetComponent<TrafficLane>();
            Assert.NotNull(lane);

            foreach(TrafficLane tl in lane.NextLanes)
            {
                Debug.Log(Vector3.Distance(lane.Waypoints[lane.Waypoints.Length-1], tl.transform.position));
            }
        }

        Assert.AreEqual(trStops.childCount, 4);

    }
}
