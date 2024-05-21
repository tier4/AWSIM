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

public class VehicleDynamicsTest
{
    // Scene handlers
    AsyncOperation sceneLoader;
    string sceneName = "VehicleDynamicsTest";
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

    // Move commands
    autoware_auto_control_msgs.msg.AckermannControlCommand moveCommand = new autoware_auto_control_msgs.msg.AckermannControlCommand()
    {
        Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
            Acceleration = 1.0f
        }
    };
    autoware_auto_control_msgs.msg.AckermannControlCommand breakCommand = new autoware_auto_control_msgs.msg.AckermannControlCommand()
    {
        Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
            Acceleration = -2.0f
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
    QoSSettings qosSettings = new QoSSettings()
    {
        ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
        DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST
    };

    

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

   [TearDown]
    public void TearDown()
    {
        EditorSceneManager.UnloadScene(scene);
    }



    [UnityTest]
    public IEnumerator VehicleDynamics_StraightMove_LowAcceleration()
    {
        // The distance travelled by car is calculated using an analytical formula:
        // 0.5 * accel * t^2
        float errorThreshold = 0.1f; // The percentage by which the final position of the ego can deviate from the analytical position due to numerical error.
        float driveTime = 4.0f; // Ego travel time
        float accel = 0.5f;
        float expectedDistance = 0.5f * accel * driveTime * driveTime;
        float acceptableErrorDistance = expectedDistance * errorThreshold;

        autoware_auto_control_msgs.msg.AckermannControlCommand moveCmd = new autoware_auto_control_msgs.msg.AckermannControlCommand()
        {
            Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
                Acceleration = accel
            }
        };

        Vector3 initPosition = egoGameObject.transform.position;

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Set Vehicle gear to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Apply acceleration
        movementPublisher.Publish(moveCmd);
        yield return new WaitForSecondsRealtime(driveTime);

        // Get ego current position
        Vector3 currPosition = egoGameObject.transform.position;
        float distance = Vector3.Distance(initPosition, currPosition);
        Debug.Log(distance + " :: " + expectedDistance);
        Assert.That(Utils.AreFloatsEqual(distance, expectedDistance, acceptableErrorDistance), Is.True);       
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_StraightMove_MiddleAcceleration()
    {
        // The distance travelled by car is calculated using an analytical formula:
        // 0.5 * accel * t^2
        float errorThreshold = 0.1f; // The percentage by which the final position of the ego can deviate from the analytical position due to numerical error.
        float driveTime = 4.0f; // Ego travel time
        float accel = 1.0f;
        float expectedDistance = 0.5f * accel * driveTime * driveTime;
        float acceptableErrorDistance = expectedDistance * errorThreshold;

        autoware_auto_control_msgs.msg.AckermannControlCommand moveCmd = new autoware_auto_control_msgs.msg.AckermannControlCommand()
        {
            Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
                Acceleration = accel
            }
        };

        Vector3 initPosition = egoGameObject.transform.position;

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Set Vehicle gear to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Apply acceleration
        movementPublisher.Publish(moveCmd);
        yield return new WaitForSecondsRealtime(driveTime);

        // Get ego current position
        Vector3 currPosition = egoGameObject.transform.position;
        float distance = Vector3.Distance(initPosition, currPosition);
        Debug.Log(distance + " :: " + expectedDistance);
        Assert.That(Utils.AreFloatsEqual(distance, expectedDistance, acceptableErrorDistance), Is.True);       
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_StraightMove_HighAcceleration()
    {
        // The distance travelled by car is calculated using an analytical formula:
        // 0.5 * accel * t^2
        float errorThreshold = 0.1f; // The percentage by which the final position of the ego can deviate from the analytical position due to numerical error.
        float driveTime = 4.0f; // Ego travel time
        float accel = 2.0f;
        float expectedDistance = 0.5f * accel * driveTime * driveTime;
        float acceptableErrorDistance = expectedDistance * errorThreshold;

        autoware_auto_control_msgs.msg.AckermannControlCommand moveCmd = new autoware_auto_control_msgs.msg.AckermannControlCommand()
        {
            Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
                Acceleration = accel
            }
        };

        Vector3 initPosition = egoGameObject.transform.position;

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Set Vehicle gear to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Apply acceleration
        movementPublisher.Publish(moveCmd);
        yield return new WaitForSecondsRealtime(driveTime);

        // Get ego current position
        Vector3 currPosition = egoGameObject.transform.position;
        float distance = Vector3.Distance(initPosition, currPosition);
        Debug.Log(distance + " :: " + expectedDistance);
        Assert.That(Utils.AreFloatsEqual(distance, expectedDistance, acceptableErrorDistance), Is.True);   
    }


    [UnityTest]
    public IEnumerator VehicleDynamics_TurnLeft()
    {
        Vector3 expectedEndPosition = new Vector3(-17.27f, -0.07f, 16.26f);
        float acceptableErrorDistance = 1.0f;

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Set Vehicle gear to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        yield return new WaitForSecondsRealtime(4.0f);

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        yield return new WaitForSecondsRealtime(4.0f);

        // Turn left by 90 deg
        movementPublisher.Publish(leftCommand);
        yield return new WaitForSecondsRealtime(4.37f);

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        yield return new WaitForSecondsRealtime(2.0f);

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        yield return new WaitForSecondsRealtime(4.0f);
        
        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Debug.Log(distanceToExpectedPosition);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_RightLeft()
    {
        Vector3 expectedEndPosition = new Vector3(17.39f, -0.07f, 16.24f);
        float acceptableErrorDistance = 1.0f;

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Set Vehicle gear to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        yield return new WaitForSecondsRealtime(4.0f);

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        yield return new WaitForSecondsRealtime(4.0f);

        // Turn left by 90 deg
        movementPublisher.Publish(rightCommand);
        yield return new WaitForSecondsRealtime(4.37f);

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        yield return new WaitForSecondsRealtime(2.0f);

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        yield return new WaitForSecondsRealtime(4.0f);

        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Debug.Log(distanceToExpectedPosition);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_UTurn()
    {
        Vector3 expectedEndPosition = new Vector3(8.844f, -0.072f, -3.955f);
        float acceptableErrorDistance = 1.0f;

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Set Vehicle gear to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        yield return new WaitForSecondsRealtime(4.0f);

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        yield return new WaitForSecondsRealtime(4.0f);

        // Turn left by 90 deg
        movementPublisher.Publish(rightCommand);
        yield return new WaitForSecondsRealtime(6.59f);

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        yield return new WaitForSecondsRealtime(2.0f);

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        yield return new WaitForSecondsRealtime(6.0f);

        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Debug.Log(distanceToExpectedPosition);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_LineChange()
    {
        Vector3 expectedEndPosition = new Vector3(2.554f, -0.072f, 28.81f);
        float acceptableErrorDistance = 1.0f;

        autoware_auto_control_msgs.msg.AckermannControlCommand leftCmd = new autoware_auto_control_msgs.msg.AckermannControlCommand()
        {
            Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
                Acceleration = 1.0f
            },
            Lateral = new autoware_auto_control_msgs.msg.AckermannLateralCommand() {
                Steering_tire_angle = 0.0872f
            }
        };

        autoware_auto_control_msgs.msg.AckermannControlCommand rightCmd = new autoware_auto_control_msgs.msg.AckermannControlCommand()
        {
            Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
                Acceleration = 1.0f
            },
            Lateral = new autoware_auto_control_msgs.msg.AckermannLateralCommand() {
                Steering_tire_angle = -0.0872f
            }
        };


        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        yield return new WaitForSeconds(0.1f);

        // Set Vehicle gear to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        yield return new WaitForSeconds(0.1f);

        // move forward
        movementPublisher.Publish(moveCommand);
        yield return new WaitForSecondsRealtime(2.0f);

        // change to right line
        movementPublisher.Publish(rightCmd);
        yield return new WaitForSecondsRealtime(2.65f);

        movementPublisher.Publish(leftCmd);
        yield return new WaitForSecondsRealtime(1.6f);

        // break
        movementPublisher.Publish(breakCommand);
        yield return new WaitForSecondsRealtime(6.0f);

        
        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Debug.Log(distanceToExpectedPosition);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

}
