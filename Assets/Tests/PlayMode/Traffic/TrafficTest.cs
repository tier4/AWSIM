using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Collections;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using AWSIM.TrafficSimulation;
using AWSIM;
using System;
using System.Collections.Generic;

public class TrafficTest
{
    // Scene handling
    AsyncOperation sceneLoader;
    string sceneName = "TrafficTest";
    Scene scene;
    AsyncOperation aOp;

    // Traffic handles
    TrafficManager trafficManager;
    TrafficTestUtils trafficTestUtils;

    [UnitySetUp]
    public IEnumerator Setup()
    {
        aOp = EditorSceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);

        yield return new WaitUntil(() => aOp.isDone);
        scene = EditorSceneManager.GetSceneByName(sceneName);
        EditorSceneManager.SetActiveScene(scene);
        Assert.NotNull(scene);

        trafficManager = GameObject.FindObjectOfType<TrafficManager>();
        Assert.NotNull(trafficManager);

        trafficTestUtils = GameObject.FindObjectOfType<TrafficTestUtils>();
        Assert.NotNull(trafficTestUtils);
        yield return null;
    }


    static int[] numberOfNPCs = new int[] { 1, 2, 3, 4 };
    [UnityTest]
    public IEnumerator RandomTrafficSpawn([ValueSource("numberOfNPCs")] int numberOfNPCs)
    {
        yield return new WaitForSeconds(1);

        var singlePrefab = trafficTestUtils.prefabs;

        Array.Resize<GameObject>(ref singlePrefab, 1);
        
        RandomTrafficSimulator randomTrafficSimulator = new RandomTrafficSimulator(
            trafficManager.gameObject,
            singlePrefab,
            trafficTestUtils.startLines,
            trafficManager.npcVehicleSimulator,
            numberOfNPCs
        );
        randomTrafficSimulator.enabled = true;

        trafficManager.AddTrafficSimulator(randomTrafficSimulator);

        yield return new WaitForSeconds(1);

        string expectedName = singlePrefab[0].name;
        Assert.AreEqual(GameObject.FindObjectsOfType<NPCVehicle>().Length, numberOfNPCs);
        foreach (var item in GameObject.FindObjectsOfType<NPCVehicle>())
        {
            Assert.That(item.name.Contains(expectedName));
        };
    }

    static int[] seedNumbers = new int[] { 0, 1, 2, -20 };
    static Dictionary<int, List<string>> expectedSpawns = new Dictionary<int, List<string>>() {
        {0, new List<string> { "Taxi", "SmallCar", "SmallCar", "SmallCar" }},
        {1, new List<string> { "Van", "Hatchback", "SmallCar", "SmallCar" }},
        {2, new List<string> { "SmallCar", "Taxi", "Taxi", "Hatchback" }},
        {-20, new List<string> { "SmallCar", "Van", "Taxi", "Taxi" }},
    };
    [UnityTest]
    public IEnumerator SeedSpawn([ValueSource("seedNumbers")] int seed)
    {
        yield return new WaitForSeconds(1);

        UnityEngine.Random.InitState(seed);
        
        RandomTrafficSimulator randomTrafficSimulator = new RandomTrafficSimulator(
            trafficManager.gameObject,
            trafficTestUtils.prefabs,
            trafficTestUtils.startLines,
            trafficManager.npcVehicleSimulator,
            trafficTestUtils.prefabs.Length
        );
        randomTrafficSimulator.enabled = true;

        trafficManager.AddTrafficSimulator(randomTrafficSimulator);

        yield return new WaitForSeconds(1);

        Assert.AreEqual(GameObject.FindObjectsOfType<NPCVehicle>().Length, trafficTestUtils.prefabs.Length);

        int index = 0;
        foreach (var item in GameObject.FindObjectsOfType<NPCVehicle>())
        {
            Assert.That(item.name.Contains(expectedSpawns[seed][index]));
            index++;
        }
    }

    [UnityTest]
    public IEnumerator Despawn()
    {
        yield return new WaitForSeconds(1);

        var singlePrefab = trafficTestUtils.prefabs;

        Array.Resize<GameObject>(ref singlePrefab, 1);
        
        RandomTrafficSimulator randomTrafficSimulator = new RandomTrafficSimulator(
            trafficManager.gameObject,
            singlePrefab,
            trafficTestUtils.startLines,
            trafficManager.npcVehicleSimulator,
            4
        );
        randomTrafficSimulator.enabled = true;

        trafficManager.AddTrafficSimulator(randomTrafficSimulator);

        yield return new WaitForSeconds(10);

        Assert.AreEqual(GameObject.FindObjectsOfType<NPCVehicle>().Length, 0);
    }

    [TearDown]
    public void TearDown() 
    {
        EditorSceneManager.UnloadScene(scene);
    }
}
