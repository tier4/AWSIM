using NUnit.Framework;
using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using System.Collections;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools.Utils;
using AWSIM;
using ROS2;

public class EgoTest
{
    // Scene handling
    AsyncOperation sceneLoader;
    string sceneName = "EgoTest";
    Scene scene;
    AsyncOperation aOp;

    // Ego handlers
    GameObject egoGameObject;
    Vehicle egoVehicle;
    VehicleRosInput egoRosInput;

    ROS2.IPublisher<autoware_auto_vehicle_msgs.msg.GearCommand> gearCommandPublisher;
    ROS2.IPublisher<autoware_auto_control_msgs.msg.AckermannControlCommand> movementPublisher;

    // Gear commands
    autoware_auto_vehicle_msgs.msg.GearCommand parkGearCommand = new autoware_auto_vehicle_msgs.msg.GearCommand()
    {
        Command = autoware_auto_vehicle_msgs.msg.GearCommand.PARK
    };
    autoware_auto_vehicle_msgs.msg.GearCommand driveGearCommand = new autoware_auto_vehicle_msgs.msg.GearCommand()
    {
        Command = autoware_auto_vehicle_msgs.msg.GearCommand.DRIVE
    };
    autoware_auto_vehicle_msgs.msg.GearCommand reverseGearCommand = new autoware_auto_vehicle_msgs.msg.GearCommand()
    {
        Command = autoware_auto_vehicle_msgs.msg.GearCommand.REVERSE
    };
    autoware_auto_control_msgs.msg.AckermannControlCommand moveCommand = new autoware_auto_control_msgs.msg.AckermannControlCommand()
    {
        Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
            Acceleration = 1.0f
        }
    };
    autoware_auto_control_msgs.msg.AckermannControlCommand leftCommand = new autoware_auto_control_msgs.msg.AckermannControlCommand()
    {
        Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
            Acceleration = 1.0f
        },
        Lateral = new autoware_auto_control_msgs.msg.AckermannLateralCommand() {
            Steering_tire_angle = 0.6f
        }
    };
    autoware_auto_control_msgs.msg.AckermannControlCommand rightCommand = new autoware_auto_control_msgs.msg.AckermannControlCommand()
    {
        Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
            Acceleration = 1.0f
        },
        Lateral = new autoware_auto_control_msgs.msg.AckermannLateralCommand() {
            Steering_tire_angle = -0.6f
        }
    };

    // Shared settings
    float testDuration = 2.0f;
    int movementCommands = 200;
    QoSSettings qosSettings = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST
    };

    
    Vector3EqualityComparer v3Comparer;

    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        // Comparers
        v3Comparer = new Vector3EqualityComparer(10e-6f);
    }

    [UnitySetUp]
    public IEnumerator Setup()
    {
        aOp = EditorSceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);

        yield return new WaitUntil(() => aOp.isDone);
        scene = EditorSceneManager.GetSceneByName(sceneName);
        EditorSceneManager.SetActiveScene(scene);

        Assert.NotNull(scene);

        egoVehicle = GameObject.FindObjectOfType<AWSIM.Vehicle>();
        egoGameObject = egoVehicle.gameObject;
        egoRosInput = egoGameObject.GetComponent<VehicleRosInput>();

        Assert.NotNull(egoRosInput);
        Assert.NotNull(egoVehicle);

        // Publishers
        string gearChangeTopic = egoRosInput.GetPrivateFieldValue<string>("gearCommandTopic");
        gearCommandPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_vehicle_msgs.msg.GearCommand>(
            gearChangeTopic,
            qosSettings.GetQoSProfile()
        );

        string movementTopic = egoRosInput.GetPrivateFieldValue<string>("ackermannControlCommandTopic");
        movementPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_control_msgs.msg.AckermannControlCommand>(
            movementTopic,
            qosSettings.GetQoSProfile()
        );
        yield return null;
    }

    [UnityTest]
    public IEnumerator TurningLeft()
    {
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);
        for (int i = 0; i < movementCommands; i++)
        {
            movementPublisher.Publish(leftCommand);
            yield return new WaitForFixedUpdate();
        }

        Assert.That(egoGameObject.transform.position.z > 1.0f);
        Assert.That(egoGameObject.transform.position.x < 0.0f);
        yield return null;
    }

    [UnityTest]
    public IEnumerator TurningRight()
    {
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);
        for (int i = 0; i < movementCommands; i++)
        {
            movementPublisher.Publish(rightCommand);
            yield return new WaitForFixedUpdate();
        }

        Assert.That(egoGameObject.transform.position.z > 1.0f);
        Assert.That(egoGameObject.transform.position.x > 0.0f);
        yield return null;
    }

    [UnityTest]
    public IEnumerator GearChanging()
    {
        Vector3 egoInitialPosition = egoGameObject.transform.position; 
        
        // Vehicle should not move when gear is set to PARK.
        Debug.Log("Testing PARK gear.");
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);
        for (int i = 0; i < movementCommands; i++)
        {
            movementPublisher.Publish(moveCommand);
            yield return new WaitForFixedUpdate();
        }

        Assert.That(egoInitialPosition, Is.EqualTo(egoGameObject.transform.position).Using(v3Comparer));

        // Vehicle should move forward when the gear is set to DRIVE and the acceleration is >0.
        Debug.Log("Testing DRIVE gear.");
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);
        for (int i = 0; i < movementCommands; i++)
        {
            movementPublisher.Publish(moveCommand);
            yield return new WaitForFixedUpdate();
        }
        Assert.That(egoGameObject.transform.position.z > 1.0f);

        // Vehicle go back when reverse gear is on.
        Debug.Log("Testing REVERSE gear.");
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);
        var egoLastPosition = egoGameObject.transform.position;
        gearCommandPublisher.Publish(reverseGearCommand);
        yield return new WaitForSeconds(0.1f);
        for (int i = 0; i < movementCommands; i++)
        {
            movementPublisher.Publish(moveCommand);
            yield return new WaitForFixedUpdate();
        }
        Assert.That(egoGameObject.transform.position.z < egoLastPosition.z);
    }

    [TearDown]
    public void TearDown()
    {
        EditorSceneManager.UnloadScene(scene);
    }
}
