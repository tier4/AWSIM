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
    /// Test Outline:
    ///     - A given number of NPCs is going to be spawned into the scene.
    /// Test Target:
    ///     - Check if the TrafficManager script spawns the correct number of NPCs.
    ///     - Verify that the NPCs spawned are of the expected type.
    /// Expected Result:
    ///     - The Traffic Manager spawns the exact number of NPCs as specified.
    ///     - Spawned NPC matches the expected prefab type.
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
    /// Test Outline:
    ///     - The NPCs vehicle are going to be spawned for the given seed number.
    /// Test Target:
    ///     - Check if the TrafficManager script spawns NPCs that correspond to the given seed number.
    /// Expected Result:
    ///     - Each spawned NPC matches the type expected based on the seed and the 'expectedSpawns' list.
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
    /// Test Outline:
    ///     - A single NPC is added to the scene and despawns after 10 seconds.
    /// Test Target:
    ///     - Verify that the TrafficManager script despawns NPCs correctly.
    /// Expected Result:
    ///     - There should be no NPCs in the scene after 10 seconds.
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
    /// Test Outline:
    ///     - A single NPC enters the intersection when the GREEN light is on.
    ///     - The synchronization between the traffic light timing and the NPC approaching the intersection is set 
    ///         to ensure that the NPC encounter the GREEN light.
    /// Test Target:
    ///     - Test if the functionality of the TrafficManager, which decides if NPCs can pass through the intersection 
    ///         with the traffic light status taken into account, works correctly.
    /// Expected Result:
    ///     - The NPC vehicle passes through the intersection smoothly without stopping.
    ///     - During each second of movement, the NPC maintains a speed greater than 0f.
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
    /// Test Outline:
    ///     - A single NPC enters the intersection when the RED light is on.
    ///     - The synchronization between the traffic light timing and the NPC approaching the intersection is set 
    ///         to ensure that the NPC encounter the RED light.
    /// Test Target:
    ///     - Test if the functionality of the TrafficManager, which decides if NPCs can pass through the intersection 
    ///         with the traffic light status taken into account, works correctly.
    /// Expected Result:
    ///     - The NPC vehicle comes to a complete stop at the intersection upon encountering the RED light.
    ///     - The NPC remains stationary until the traffic light turns GREEN, indicating it can proceed.
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
    /// Test Outline:
    ///     - Two NPC vehicles enter the intersection from opposite sides while both have a GREEN light.
    ///     - One NPC is preparing to turn right, and the other is moving straight through the intersection.
    ///     - The traffic light timing and NPCs synchronization ensure that both encounter the GREEN light,
    ///         with the turning vehicle yielding to the straight-moving vehicle.
    /// Test Target:
    ///     - Test if the functionality of the TrafficManager, which decides if NPCs can proceed through
    ///         the intersection with the traffic light status taken into account, works correctly.
    ///     - Test if the functionality of the TrafficManager, which decides the yield rules at 
    ///         the intersection, works correctly.
    /// Expected Result:
    ///     - The NPC turning right yields to the NPC moving straight through the intersection.
    ///     - The NPC moving straight through the intersection proceeds without stopping.
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
