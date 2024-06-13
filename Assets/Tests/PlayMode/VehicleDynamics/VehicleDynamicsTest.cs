using NUnit.Framework;
using UnityEngine;
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
    string sceneName = "VehicleDynamicsTest";
    Scene scene;
    PhysicsScene physicsScene;

    // Ego handlers
    GameObject egoGameObject;
    Vehicle egoVehicle;
    VehicleRos2Input egoRosInput;

    // ROS publishers
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

    // Turn commands
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

    private float acceptableErrorDistance = 0.01f;


    // --- TEST LIFE CYCLE ---//

    [UnitySetUp]
    public IEnumerator Setup()
    {
        yield return LoadSceneAsync();
        yield return GetEgoComponents();
        yield return CreateEgoCommonPublihers();
        yield return new WaitForFixedUpdate();
    }

    private IEnumerator LoadSceneAsync()
    {
        LoadSceneParameters parameters = new LoadSceneParameters(LoadSceneMode.Single, LocalPhysicsMode.Physics3D);
        AsyncOperation aOp = EditorSceneManager.LoadSceneAsync(sceneName, parameters);

        yield return new WaitUntil(() => aOp.isDone);
        scene = EditorSceneManager.GetSceneByName(sceneName);
        EditorSceneManager.SetActiveScene(scene);

        physicsScene = PhysicsSceneExtensions.GetPhysicsScene(scene); 

        Assert.NotNull(scene);
        Assert.NotNull(physicsScene);
    }

    private IEnumerator GetEgoComponents()
    {
        egoVehicle = GameObject.FindObjectOfType<AWSIM.Vehicle>();
        egoGameObject = egoVehicle.gameObject;
        egoRosInput = egoGameObject.GetComponentInChildren<VehicleRos2Input>();

        Assert.NotNull(egoRosInput);
        Assert.NotNull(egoVehicle);

        yield return new WaitForFixedUpdate();
    }

    private IEnumerator CreateEgoCommonPublihers()
    {
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
        yield return new WaitForFixedUpdate();
    }


    [UnityTearDown]
    public IEnumerator TearDown()
    {
        yield return new WaitForFixedUpdate();
        yield return RemoveEgoCommonPublishers();
        yield return RemoveEgoVehicle();
    }
    
    private IEnumerator RemoveEgoVehicle()
    {
        Rigidbody rb = egoVehicle.GetComponent<Rigidbody>();
        rb.isKinematic = true;

        physicsScene.Simulate(Time.fixedDeltaTime);
        yield return new WaitForFixedUpdate();

        GameObject.DestroyImmediate(egoVehicle.gameObject);
        egoVehicle = null;
        egoGameObject = null;
        egoRosInput = null;

        for(int i=0; i<10; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }
    }

    private IEnumerator RemoveEgoCommonPublishers()
    {
        SimulatorROS2Node.RemovePublisher<autoware_auto_control_msgs.msg.AckermannControlCommand>(movementPublisher);
        SimulatorROS2Node.RemovePublisher<autoware_auto_vehicle_msgs.msg.GearCommand>(gearCommandPublisher);

        yield return new WaitForFixedUpdate();
    }



    // --- TEST ROUTINES --- //

    [UnityTest]
    public IEnumerator VehicleDynamics_StraightMove_LowAcceleration()
    {
        Vector3 expectedEndPosition = new Vector3(0.0005525741f, -0.07694209f, 3.983599f);
        float accel = 0.5f;

        autoware_auto_control_msgs.msg.AckermannControlCommand moveCmd = new autoware_auto_control_msgs.msg.AckermannControlCommand()
        {
            Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
                Acceleration = accel
            }
        };


        // Initial physics simulation steps
        for(int i=0; i<10; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle gear to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Apply acceleration
        movementPublisher.Publish(moveCmd);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        
        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_StraightMove_MiddleAcceleration()
    {
        Vector3 expectedEndPosition = new Vector3(0.003112176f, -0.07788333f, 7.9672f);
        float accel = 1.0f;

        autoware_auto_control_msgs.msg.AckermannControlCommand moveCmd = new autoware_auto_control_msgs.msg.AckermannControlCommand()
        {
            Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
                Acceleration = accel
            }
        };


        // Initial physics simulation steps
        for(int i=0; i<10; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Apply acceleration
        movementPublisher.Publish(moveCmd);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }


        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);  
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_StraightMove_HighAcceleration()
    {
        Vector3 expectedEndPosition = new Vector3(0.01421936f, -0.07976329f, 15.9344f);  
        float accel = 2.0f;

        autoware_auto_control_msgs.msg.AckermannControlCommand moveCmd = new autoware_auto_control_msgs.msg.AckermannControlCommand()
        {
            Longitudinal = new autoware_auto_control_msgs.msg.LongitudinalCommand() {
                Acceleration = accel
            }
        };


        // Initial physics simulation steps
        for(int i=0; i<10; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Apply acceleration
        movementPublisher.Publish(moveCmd);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }


        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_TurnLeft()
    {
        Vector3 expectedEndPosition = new Vector3(-17.19837f, -0.07222256f, 16.39548f);


        // Initial physics simulation steps
        for(int i=0; i<10; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Turn left by 90 deg
        movementPublisher.Publish(leftCommand);
        for(int i=0; i<262; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // move forward for 2 seconds
        movementPublisher.Publish(moveCommand);
        for(int i=0; i<120; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }


        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_TurnRight()
    {
        Vector3 expectedEndPosition = new Vector3(17.31582f, -0.07222262f, 16.27713f);


        // Initial physics simulation steps
        for(int i=0; i<10; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Turn left by 90 deg
        movementPublisher.Publish(rightCommand);
        for(int i=0; i<262; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // move forward for 2 seconds
        movementPublisher.Publish(moveCommand);
        for(int i=0; i<120; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }


        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_UTurn()
    {
        Vector3 expectedEndPosition = new Vector3(9.046477f, -0.07222325f, -3.879607f);


        // Initial physics simulation steps
        for(int i=0; i<10; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // move forward for 4 seconds
        movementPublisher.Publish(moveCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Turn by 180 deg
        movementPublisher.Publish(rightCommand);
        for(int i=0; i<395; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // move forward for 2 seconds
        movementPublisher.Publish(moveCommand);
        for(int i=0; i<120; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // break for 4 seconds
        movementPublisher.Publish(breakCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }


        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

    [UnityTest]
    public IEnumerator VehicleDynamics_LineChange()
    {
        Vector3 expectedEndPosition = new Vector3(2.553552f, -0.07222325f, 28.81023f);

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


        // Initial physics simulation steps
        for(int i=0; i<10; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to PARK
        gearCommandPublisher.Publish(parkGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // Set Vehicle to DRIVE
        gearCommandPublisher.Publish(driveGearCommand);
        for(int i=0; i<6; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // move forward
        movementPublisher.Publish(moveCommand);
        for(int i=0; i<120; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // change to right line
        movementPublisher.Publish(rightCmd);
        for(int i=0; i<159; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        movementPublisher.Publish(leftCmd);
        for(int i=0; i<96; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        // break
        movementPublisher.Publish(breakCommand);
        for(int i=0; i<240; i++)
        {
            physicsScene.Simulate(Time.fixedDeltaTime);
            yield return new WaitForFixedUpdate();
        }

        float distanceToExpectedPosition = Vector3.Distance(expectedEndPosition, egoGameObject.transform.position);
        Assert.That(Utils.AreFloatsEqual(0.0f, distanceToExpectedPosition, acceptableErrorDistance), Is.True);
    }

}
