using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Collections;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using AWSIM.TrafficSimulation;
using AWSIM;
using AWSIM.Tests;
using System;
using System.Collections.Generic;

/// <summary>
/// The TrafficTest class is responsible for managing the testing of the Traffic Manager feature.
/// It loads the required scene and runs tests to validate the functionality of the Traffic Manager.
/// </summary>
public class TrafficTest
{
    // Scene handling
    string sceneName = "TrafficTest";
    Scene scene;
    AsyncOperation aOp;

    // Traffic handles
    TrafficManager trafficManager;
    TrafficTestUtils trafficTestUtils;
    TrafficTestEnvironmentCollection trafficEnvironmentCollection;
    TrafficTestScenarioCollection trafficTestScenarioCollection;


    // --- TEST LIFE CYCLE ---//

    /// <summary>
    /// Method called by Unity at the start of each test.
    /// This method handles scene loading and the collection of traffic system components.
    /// </summary>
    [UnitySetUp]
    public IEnumerator Setup()
    {
        aOp = EditorSceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Single);

        yield return new WaitUntil(() => aOp.isDone);
        scene = EditorSceneManager.GetSceneByName(sceneName);
        EditorSceneManager.SetActiveScene(scene);
        Assert.NotNull(scene);

        trafficManager = GameObject.FindObjectOfType<TrafficManager>();
        Assert.NotNull(trafficManager);

        trafficTestUtils = GameObject.FindObjectOfType<TrafficTestUtils>();
        Assert.NotNull(trafficTestUtils);

        trafficEnvironmentCollection = GameObject.FindObjectOfType<TrafficTestEnvironmentCollection>();
        Assert.NotNull(trafficEnvironmentCollection);

        trafficTestScenarioCollection = GameObject.FindObjectOfType<TrafficTestScenarioCollection>();
        Assert.NotNull(trafficTestScenarioCollection);

        yield return null;
    }

    /// <summary>
    /// Activate environment objects necessary for a specific test.
    /// </summary>
    private IEnumerator SetupEnvironment(string testName)
    {
        Camera.main.transform.position = trafficEnvironmentCollection.GetCameraPosition(testName);
        Camera.main.transform.eulerAngles = trafficEnvironmentCollection.GetCameraRotation(testName);

        trafficEnvironmentCollection.SelectEnvironment(testName);
        yield return null;
    }

    /// <summary>
    /// Method called by Unity at the end of each test.
    /// </summary>
    [UnityTearDown]
    public IEnumerator TearDown() 
    {
        yield return null;
    }


    // --- TEST ROUTINES --- //

    static int[] numberOfNPCs = new int[] { 1, 2, 3, 4 };
    
    /// <summary>
    /// A test to validate if the traffic manager spawns the correct number of NPCs, and if the expected prefab is spawned.
    /// </summary>
    [UnityTest]
    public IEnumerator RandomTrafficSpawn([ValueSource("numberOfNPCs")] int numberOfNPCs)
    {
        string testScenario = "Straight_Lanes";
        yield return SetupEnvironment(testScenario);

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
    
    /// <summary>
    /// A test to validate if the traffic manager spawns the correct type of NPCs for a given seed number.
    /// </summary>
    [UnityTest]
    public IEnumerator SeedSpawn([ValueSource("seedNumbers")] int seed)
    {
        string testScenario = "Straight_Lanes";
        yield return SetupEnvironment(testScenario);

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

    /// <summary>
    /// A test to validate if the NPCs are correctly despawned.
    /// </summary>
    [UnityTest]
    public IEnumerator Despawn()
    {
        string testScenario = "Straight_Lanes";
        yield return SetupEnvironment(testScenario);

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

    /// <summary>
    /// A test to validate the correctness of the traffic simulator behavior. In this test,
    /// a single NPC vehicle enters the intersection when the GREEN light turns on.
    /// The expected behavior assumes that the NPC vehicle moves through the intersection without stopping.
    /// </summary>
    [UnityTest]
    public IEnumerator TrafficManager_MoveStraight_GreenLight()
    {
        string testScenario = "Intersections";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        TestRouteTrafficConfiguration[] routeConfigs = trafficTestScenarioCollection.
            GetTestRouteTrafficConfigs("MoveStraight_GreenLight");
        Assert.NotNull(routeConfigs);

        foreach (var route in routeConfigs)
        {
            RouteTrafficSimulator routeTs = new RouteTrafficSimulator(
                trafficManager.gameObject,
                route.Config.npcPrefabs,
                route.Config.route,
                trafficManager.npcVehicleSimulator,
                route.Config.maximumSpawns
            );

            routeTs.enabled = true;
            trafficManager.AddTrafficSimulator(routeTs);
        }

        yield return new WaitForSecondsRealtime(1.0f);

        NPCVehicle[] npcs = GameObject.FindObjectsOfType<NPCVehicle>();
        Assert.NotNull(npcs);
        Assert.AreEqual(npcs.Length, 1);

        // check for 15 seconds that the vehicle is moving (vehicle speed should be greater than 0).
        for(int i=0; i<15; i++)
        {
            float speed = npcs[0].GetPrivateFieldValue<float>("speed");
            Assert.Greater(speed, 0f);
            yield return new WaitForSecondsRealtime(1.0f);
        }
    }

    /// <summary>
    /// A test to validate the correctness of the traffic simulator behavior. In this test,
    /// a single NPC vehicle enters the intersection when the RED light is on.
    /// The expected behavior assumes that the NPC vehicle stops at the intersection and waits for the green light.
    /// </summary>
    [UnityTest]
    public IEnumerator TrafficManager_MoveStraight_RedLight()
    {
        string testScenario = "Intersections";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        TestRouteTrafficConfiguration[] routeConfigs = trafficTestScenarioCollection.
            GetTestRouteTrafficConfigs("MoveStraight_RedLight");
        Assert.NotNull(routeConfigs);

        foreach (var route in routeConfigs)
        {
            RouteTrafficSimulator routeTs = new RouteTrafficSimulator(
                trafficManager.gameObject,
                route.Config.npcPrefabs,
                route.Config.route,
                trafficManager.npcVehicleSimulator,
                route.Config.maximumSpawns
            );

            routeTs.enabled = true;
            trafficManager.AddTrafficSimulator(routeTs);
        }

        yield return new WaitForSecondsRealtime(1.0f);

        NPCVehicle[] npcs = GameObject.FindObjectsOfType<NPCVehicle>();
        Assert.NotNull(npcs);
        Assert.AreEqual(npcs.Length, 1);

        // After spawning, the NPC should have a speed greater than 0. 
        float speed = npcs[0].GetPrivateFieldValue<float>("speed");
        Assert.Greater(speed, 0f);

        // After 20 seconds NPCs should arrive intersection. There should be a red light for the NPC. Therefore the NPC should not be moving.
        yield return new WaitForSecondsRealtime(20.0f);
        speed = npcs[0].GetPrivateFieldValue<float>("speed");
        Assert.LessOrEqual(Mathf.Abs(speed), 0.00001f);

        // After next 20 seconds the traffic light should changed to green. Therefore the NPC should be moving.
        yield return new WaitForSecondsRealtime(20.0f);
        speed = npcs[0].GetPrivateFieldValue<float>("speed");
        Assert.Greater(speed, 0f);
    }

    /// <summary>
    /// A test to validate the correctness of traffic simulator behavior. In this test, two NPC vehicles
    /// enter the intersection when the GREEN light is on. These NPCs enter the intersection from opposite sides,
    /// with one vehicle about to turn right and the other moving straight through the intersection.
    /// Expected behavior:
    ///  - the first vehicle (turning right) must yield to the second vehicle.
    ///  - the second vehicle (moving straight) should proceed through the intersection without stopping.
    /// </summary>
    [UnityTest]
    public IEnumerator TrafficManager_Oposite_GreeLight()
    {
        string testScenario = "Intersections";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        TestRouteTrafficConfiguration[] routeConfigs = trafficTestScenarioCollection.
            GetTestRouteTrafficConfigs("Oposite_GreenLight");
        Assert.NotNull(routeConfigs);

        string firstVehicleExpectedName = routeConfigs[0].Config.npcPrefabs[0].name;

        foreach (var route in routeConfigs)
        {
            RouteTrafficSimulator routeTs = new RouteTrafficSimulator(
                trafficManager.gameObject,
                route.Config.npcPrefabs,
                route.Config.route,
                trafficManager.npcVehicleSimulator,
                route.Config.maximumSpawns
            );

            routeTs.enabled = true;
            trafficManager.AddTrafficSimulator(routeTs);
        }

        yield return new WaitForSecondsRealtime(1.0f);

        NPCVehicle[] npcs = GameObject.FindObjectsOfType<NPCVehicle>();
        Assert.NotNull(npcs);
        Assert.AreEqual(npcs.Length, 2);

        // Identify the NPC vehicle that is about to turn right, this is the vehicle whose name contains the firstVehicleExpectName. 
        NPCVehicle firstVehicle = default;
        NPCVehicle secondVehicle = default;
        if(npcs[0].name.Contains(firstVehicleExpectedName))
        {
            firstVehicle = npcs[0];
            secondVehicle = npcs[1];
        }
        else
        {
            firstVehicle = npcs[1];
            secondVehicle = npcs[0];
        }

        Assert.NotNull(firstVehicle);
        Assert.NotNull(secondVehicle);

        // For the 15 seconds, check if the vehicles behave as expected.
        for(int i=0; i<15; i++)
        {
            // The second vehicle should move continuously without stopping during the test.
            float speed = secondVehicle.GetPrivateFieldValue<float>("speed");
            Assert.Greater(speed, 0f);
            
            // The first vehicle should give way to the second, so there is time when the first vehicle is not moving.
            if(i == 0)
            {
                speed = firstVehicle.GetPrivateFieldValue<float>("speed");
                Assert.Greater(speed, 0f);
            }
            else if(i == 5)
            {
                speed = firstVehicle.GetPrivateFieldValue<float>("speed");
                Assert.LessOrEqual(Mathf.Abs(speed), 0.00001f);
            }
            else if(i == 14)
            {
                speed = firstVehicle.GetPrivateFieldValue<float>("speed");
                Assert.Greater(speed, 0f);
            }

            yield return new WaitForSecondsRealtime(1.0f);
        }
    }
}
