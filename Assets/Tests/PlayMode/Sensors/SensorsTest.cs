using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools.Utils;
using AWSIM;
using AWSIM.Tests;

/// <summary>
/// The SensorTest class manages the testing of sensor features.
/// It is responsible for loading the required scene and executing tests of the sensors.
/// </summary>
public class SensorsTest
{
    // Scene handling
    string sceneName = "SensorsTest";
    Scene scene;
    AsyncOperation aOp;

    // Comparers
    Vector3EqualityComparer v3Comparer;

    // Shared settings
    float testDuration = 2.0f;

    // Shared components
    private TestObjectEnvironmentCollection testObjectEnvironmentCollection;

    // GNSS
    List<geometry_msgs.msg.PoseStamped> poseMessages;
    List<geometry_msgs.msg.PoseWithCovarianceStamped> poseWithCovarianceMessages;

    // LiDAR
    List<sensor_msgs.msg.PointCloud2> lidarMessages;

    // LiDAR & Radar
    ROS2.ISubscription<sensor_msgs.msg.PointCloud2> pointCloudSubscription;
    List<sensor_msgs.msg.PointCloud2> pointCloudMessages;

    // IMU
    List<sensor_msgs.msg.Imu> imuMessages;



     // --- TEST LIFE CYCLE ---//

    /// <summary>
    /// A method called by Unity before all sensor tests, responsible for loading the test scene.
    /// </summary>
    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        // Scene async operation
        aOp = EditorSceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);

        // Comparers
        v3Comparer = new Vector3EqualityComparer(10e-6f);
    }

    /// <summary>
    /// Method called by Unity at the beginning of each test.
    /// </summary>
    [UnitySetUp]
    public IEnumerator Setup()
    {
        yield return new WaitUntil(() => aOp.isDone);
        scene = EditorSceneManager.GetSceneByName(sceneName);
        EditorSceneManager.SetActiveScene(scene);

        Assert.NotNull(scene);

        // Get components
        testObjectEnvironmentCollection = GameObject.FindObjectOfType<TestObjectEnvironmentCollection>();
        Assert.NotNull(testObjectEnvironmentCollection);
        testObjectEnvironmentCollection.DisableAll();

        // Init vars
        yield return InitVars();
        yield return null;
    }

    /// <summary>
    /// Initialize the variables required for the tests.
    /// </summary>
    private IEnumerator InitVars()
    {
        // GNSS
        if(poseMessages != null)
        {
            poseMessages.Clear();
        }
        poseMessages = new List<geometry_msgs.msg.PoseStamped>();

        if(poseWithCovarianceMessages != null)
        {
            poseWithCovarianceMessages.Clear();
        }
        poseWithCovarianceMessages = new List<geometry_msgs.msg.PoseWithCovarianceStamped>();

        // IMU
        if(imuMessages != null)
        {
            imuMessages.Clear();
        }
        imuMessages = new List<sensor_msgs.msg.Imu>();

        //LiDAR
        if(lidarMessages != null)
        {
            lidarMessages.Clear();
        }
        lidarMessages = new List<sensor_msgs.msg.PointCloud2>();

        // LiDAR & Radar
        if(pointCloudMessages != null)
        {
            pointCloudMessages.Clear();
        }
        pointCloudMessages = new List<sensor_msgs.msg.PointCloud2>();

        yield return null;
    }

    /// <summary>
    /// Activate the tested object and any environment objects required for the test.
    /// </summary>
    private IEnumerator SetupEnvironment(string testName)
    {
        GameObject environment = testObjectEnvironmentCollection.GetTestEnvironment(testName);
        if(environment != null)
        {
            environment.SetActive(true);
        }

        GameObject testObject = testObjectEnvironmentCollection.GetTestObject(testName);
        if(testObject != null)
        {
            testObject.SetActive(true);
        }
        
        yield return null;
    }

    /// <summary>
    /// Method called by Unity after all sensor tests, responsible for unloading the test scene.
    /// </summary>
    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        EditorSceneManager.UnloadScene(scene);
    }



    // --- TEST ROUTINES --- //

    /// <summary>
    /// Test to validate that the LiDAR is publishing messages and that they are occurring at the expected frequency.
    /// </summary>
    [UnityTest]
    public IEnumerator LiDAR()
    {
        string testScenario = "LidarVLP16_Sphere5m";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        RGLUnityPlugin.LidarSensor lidarSensor = GameObject.FindObjectOfType<RGLUnityPlugin.LidarSensor>();
        Assert.NotNull(lidarSensor);
        RglLidarPublisher lidarRos2Publisher = lidarSensor.GetComponent<RglLidarPublisher>();

        Assert.AreEqual((byte)lidarRos2Publisher.qos.reliabilityPolicy , (byte)ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);

        Assert.NotZero(lidarRos2Publisher.pointCloud2Publishers.Count);

        // Test all LiDAR PointCloud2 publishers
        foreach (var publisher in lidarRos2Publisher.pointCloud2Publishers)
        {
            CreatePointCloud2Subscription(publisher);
            yield return new WaitForSeconds(testDuration);

            Assert.IsNotEmpty(pointCloudMessages);
            Assert.AreEqual(pointCloudMessages.Count, (int)(testDuration * lidarSensor.AutomaticCaptureHz));
        }
    }

    /// <summary>
    /// Test to validate that the Radar is publishing messages and that they are occurring at the expected frequency.
    /// </summary>
    [UnityTest]
    public IEnumerator Radar()
    {
        string testScenario = "SmartmicroMediumRange_Sphere5m";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        RGLUnityPlugin.RadarSensor radarSensor = GameObject.FindObjectOfType<RGLUnityPlugin.RadarSensor>();
        Assert.NotNull(radarSensor);
        RglLidarPublisher radarRos2Publisher = radarSensor.GetComponent<RglLidarPublisher>();

        Assert.AreEqual((byte)radarRos2Publisher.qos.reliabilityPolicy , (byte)ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);

        Assert.NotZero(radarRos2Publisher.pointCloud2Publishers.Count);

        // Test all Radar PointCloud2 publishers
        foreach (var publisher in radarRos2Publisher.pointCloud2Publishers)
        {
            CreatePointCloud2Subscription(publisher);
            yield return new WaitForSeconds(testDuration);

            Assert.IsNotEmpty(pointCloudMessages);
            Assert.AreEqual(pointCloudMessages.Count, (int)(testDuration * radarSensor.automaticCaptureHz));
        }
    }

    /// <summary>
    /// Test to validate that the GNSS is publishing valid messages and that they are occurring at the expected frequency.
    /// </summary>
    [UnityTest]
    public IEnumerator GNSS()
    {
        string testScenario = "Gnss";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        GnssSensor gnssSensor = GameObject.FindObjectOfType<GnssSensor>();
        Assert.NotNull(gnssSensor);
        GnssRos2Publisher gnssRos2Publisher = gnssSensor.GetComponent<GnssRos2Publisher>();

        ROS2.ISubscription<geometry_msgs.msg.PoseStamped> gnssPoseSubscription = SimulatorROS2Node.CreateSubscription<geometry_msgs.msg.PoseStamped>(
            gnssRos2Publisher.poseTopic, msg =>
        {
            poseMessages.Add(msg);
        });
        ROS2.ISubscription<geometry_msgs.msg.PoseWithCovarianceStamped> gnssPoseWithCovarianceSubscription = SimulatorROS2Node.CreateSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(
            gnssRos2Publisher.poseWithCovarianceStampedTopic, msg =>
        {
            poseWithCovarianceMessages.Add(msg);
        });

        yield return new WaitForSeconds(testDuration);

        Assert.IsNotEmpty(poseMessages);
        Assert.IsNotEmpty(poseWithCovarianceMessages);

        poseMessages.ForEach(pose =>
        {
            var poseVec = new Vector3(
                (float)pose.Pose.Position.X,
                (float)pose.Pose.Position.Y,
                (float)pose.Pose.Position.Z
            );
            Assert.That(poseVec, Is.EqualTo(Vector3.zero).Using(v3Comparer));
        });

        poseWithCovarianceMessages.ForEach(pose =>
        {
            var poseVec = new Vector3(
                (float)pose.Pose.Pose.Position.X,
                (float)pose.Pose.Pose.Position.Y,
                (float)pose.Pose.Pose.Position.Z
            );
            Assert.That(poseVec, Is.EqualTo(Vector3.zero).Using(v3Comparer));
        });

        Assert.AreEqual(poseMessages.Count, (int)(testDuration * gnssSensor.OutputHz));
        Assert.AreEqual(poseWithCovarianceMessages.Count, (int)(testDuration * gnssSensor.OutputHz));

        SimulatorROS2Node.RemoveSubscription<geometry_msgs.msg.PoseStamped>(gnssPoseSubscription);
        SimulatorROS2Node.RemoveSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(gnssPoseWithCovarianceSubscription);
    }

    /// <summary>
    /// Test to validate that the NOT moving IMU is publishing valid messages and that they are occurring at the expected frequency.
    /// </summary>
    [UnityTest]
    public IEnumerator IMU()
    {
        string testScenario = "Imu";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        ImuSensor imuSensor = GameObject.FindObjectOfType<ImuSensor>();
        Assert.NotNull(imuSensor);
        ImuRos2Publisher imuRos2Publisher = imuSensor.GetComponent<ImuRos2Publisher>();

        ROS2.ISubscription<sensor_msgs.msg.Imu> imuSubscription = SimulatorROS2Node.CreateSubscription<sensor_msgs.msg.Imu>(
            imuRos2Publisher.topic, msg =>
        {
            imuMessages.Add(msg);
        });
        yield return new WaitForSeconds(testDuration);

        Assert.IsNotEmpty(imuMessages);
        Assert.AreEqual(imuMessages.Count, (int)(testDuration * imuSensor.OutputHz));

        imuMessages.ForEach(imu =>
        {
            var dataVec = new Vector3(
                (float)imu.Linear_acceleration.X,
                (float)imu.Linear_acceleration.Y,
                (float)imu.Linear_acceleration.Z
            );
            Assert.That(dataVec, Is.EqualTo(Vector3.zero).Using(v3Comparer));
        });

        SimulatorROS2Node.RemoveSubscription<sensor_msgs.msg.Imu>(imuSubscription);
    }

    /// <summary>
    /// A test to validate that the moving IMU is publishing correct messages. The IMU sensor is subjected to a constant linear acceleration during the test,
    /// simulated by manually setting the IMU object position over several consecutive frames. The IMU position for each frame is calculated using the formula:
    /// distance = 0.5 * acceleration * (fixed delta time )^2
    /// The test includes three variants of movement along the X, Y and Z axes.
    /// </summary>
    static Vector3[] accelDirections = new Vector3[] {new Vector3(1.0f, 0.0f, 0.0f), new Vector3(0.0f, 1.0f, 0.0f), new Vector3(0.0f, 0.0f, 1.0f)};
    [UnityTest]
    public IEnumerator IMU_Acceleration([ValueSource("accelDirections")] Vector3 accelDirection)
    {
        string testScenario = "Imu";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        float accelMagnitude = 1.0f;
        Vector3 expectedAccel = AWSIM.ROS2Utility.UnityToRosPosition(accelDirection * accelMagnitude);
        Vector3EqualityComparer imuAccelComparer = new Vector3EqualityComparer(10e-3f);

        ImuSensor imuSensor = GameObject.FindObjectOfType<ImuSensor>();
        Assert.NotNull(imuSensor);
        ImuRos2Publisher imuRos2Publisher = imuSensor.GetComponent<ImuRos2Publisher>();

        // reset position of Imu Sensor
        float time = 0f;
        Vector3 prevSpeed = Vector3.zero;
        imuSensor.transform.position = Vector3.zero;

        // The IMU position must be reset to zero for at least two consecutive frames.
        for (int i=0; i<5; i++)
        {
            yield return new WaitForFixedUpdate();
        }

        ROS2.ISubscription<sensor_msgs.msg.Imu> imuSubscription = SimulatorROS2Node.
            CreateSubscription<sensor_msgs.msg.Imu>(imuRos2Publisher.topic, msg =>
        {
            imuMessages.Add(msg);
        });

        // move game object
        while(time < testDuration)
        {
            Vector3 currPosition = imuSensor.transform.position;
            Vector3 targetSpeed = prevSpeed + accelDirection * accelMagnitude * Time.fixedDeltaTime;
            Vector3 targetPosition = currPosition + 0.5f * (prevSpeed + targetSpeed) * Time.fixedDeltaTime;

            imuSensor.transform.position = targetPosition;

            time += Time.fixedDeltaTime;
            yield return new WaitForFixedUpdate();

            prevSpeed = targetSpeed;
        }

        SimulatorROS2Node.RemoveSubscription<sensor_msgs.msg.Imu>(imuSubscription);

        Assert.IsNotEmpty(imuMessages);
        
        // the first message is skipped as it looks like is not reliable, due to race condition between
        // FixedUpdate() and yield return new WaitForFixedUpdate();
        for(int i=1; i<imuMessages.Count; i++)
        {
            var dataVec = new Vector3(
                (float)imuMessages[i].Linear_acceleration.X,
                (float)imuMessages[i].Linear_acceleration.Y,
                (float)imuMessages[i].Linear_acceleration.Z
            );

            Assert.That(dataVec, Is.EqualTo(expectedAccel).Using(imuAccelComparer));
        }

        imuMessages.Clear();  
    }

    /// <summary>
    /// Test to validate if the LiDAR publishes messages at the expected rate.
    /// </summary>
    [UnityTest]
    public IEnumerator LidarVLP16_PublishRate()
    {
        string testScenario = "LidarVLP16_Sphere5m";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        RGLUnityPlugin.LidarSensor lidarSensor = GameObject.FindObjectOfType<RGLUnityPlugin.LidarSensor>();
        Assert.NotNull(lidarSensor);
        RglLidarPublisher rglLidarPublisher = lidarSensor.GetComponent<RglLidarPublisher>();
        
        ROS2.QualityOfServiceProfile qos = ConvertRGLqosToROS2qos(rglLidarPublisher.qos);

        ROS2.ISubscription<sensor_msgs.msg.PointCloud2> lidarSubscription = SimulatorROS2Node.CreateSubscription<sensor_msgs.msg.PointCloud2>(
            rglLidarPublisher.pointCloud2Publishers[0].topic, GetMessageCallback, qos);

        yield return new WaitForSeconds(testDuration);
        Assert.IsNotEmpty(lidarMessages);
        Assert.AreEqual(lidarMessages.Count, (int)testDuration * lidarSensor.AutomaticCaptureHz);

        SimulatorROS2Node.RemoveSubscription<sensor_msgs.msg.PointCloud2>(lidarSubscription);

        // callbacks
        void GetMessageCallback(sensor_msgs.msg.PointCloud2 msg)
        {
            lidarMessages.Add(msg);
        }    
    }

    /// <summary>
    /// Test to validate correctness of LiDAR Point Cloud data. During the test, the LiDAR is instantiated inside a 5m radius sphere.
    /// The test checks if all points in the point cloud data are at the expected distance from the LiDAR.
    /// </summary>
    [UnityTest]
    public IEnumerator LidarVLP16_PointCloud_Distance5m()
    {
        string testScenario = "LidarVLP16_Sphere5m";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        float expectedDistance = 5.0f;
        float expectedError = 0.001f; // procentage of expected distance

        RGLUnityPlugin.LidarSensor lidarSensor = GameObject.FindObjectOfType<RGLUnityPlugin.LidarSensor>();
        Assert.NotNull(lidarSensor);
        RglLidarPublisher rglLidarPublisher = lidarSensor.GetComponent<RglLidarPublisher>();

        // remove noise in lidar
        lidarSensor.applyDistanceGaussianNoise = false;
        lidarSensor.applyAngularGaussianNoise = false;
        lidarSensor.applyVelocityDistortion = false;
        lidarSensor.OnValidate();

        // create subcriber
        ROS2.QualityOfServiceProfile qos = ConvertRGLqosToROS2qos(rglLidarPublisher.qos);
        ROS2.ISubscription<sensor_msgs.msg.PointCloud2> lidarSubscription = SimulatorROS2Node.CreateSubscription<sensor_msgs.msg.PointCloud2>(
            rglLidarPublisher.pointCloud2Publishers[0].topic, GetMessageCallback, qos);

        yield return new WaitForSeconds(testDuration);
        Assert.IsNotEmpty(lidarMessages);

        // point cloud data 
        Vector3[] points = ConvertCloudPointToUnityPosition(lidarMessages[lidarMessages.Count - 1].Data,
            (int)lidarMessages[lidarMessages.Count - 1].Point_step);
        
        float diffMax = 0f;
        for(int i=0; i<points.Length; i++)
        {
            float diff = Mathf.Abs(expectedDistance - points[i].magnitude);
            
            // find max value
            if(diff > diffMax)
            {
                diffMax = diff;
            }
        }

        Assert.That(diffMax, Is.LessThanOrEqualTo(expectedDistance * expectedError));

        SimulatorROS2Node.RemoveSubscription<sensor_msgs.msg.PointCloud2>(lidarSubscription);

        // callbacks
        void GetMessageCallback(sensor_msgs.msg.PointCloud2 msg)
        {
            lidarMessages.Add(msg);
        }
    }

    /// <summary>
    /// Test to validate correctness of LiDAR Point Cloud data. During the test, the LiDAR is instantiated inside a 10m radius sphere.
    /// The test checks if all points in the point cloud data are at the expected distance from the LiDAR.
    /// </summary>
    [UnityTest]
    public IEnumerator LidarVLP16_PointCloud_Distance10m()
    {
        string testScenario = "LidarVLP16_Sphere10m";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        float expectedDistance = 10.0f;
        float acceptedError = 0.001f; // procentage of expected distance

        RGLUnityPlugin.LidarSensor lidarSensor = GameObject.FindObjectOfType<RGLUnityPlugin.LidarSensor>();
        Assert.NotNull(lidarSensor);
        RglLidarPublisher rglLidarPublisher = lidarSensor.GetComponent<RglLidarPublisher>();

        // remove noise in lidar
        lidarSensor.applyDistanceGaussianNoise = false;
        lidarSensor.applyAngularGaussianNoise = false;
        lidarSensor.applyVelocityDistortion = false;
        lidarSensor.OnValidate();

        // create subcriber
        ROS2.QualityOfServiceProfile qos = ConvertRGLqosToROS2qos(rglLidarPublisher.qos);
        ROS2.ISubscription<sensor_msgs.msg.PointCloud2> lidarSubscription = SimulatorROS2Node.CreateSubscription<sensor_msgs.msg.PointCloud2>(
            rglLidarPublisher.pointCloud2Publishers[0].topic, GetMessageCallback, qos);

        yield return new WaitForSeconds(testDuration);
        Assert.IsNotEmpty(lidarMessages);

        // point cloud data 
        Vector3[] points = ConvertCloudPointToUnityPosition(lidarMessages[lidarMessages.Count - 1].Data,
            (int)lidarMessages[lidarMessages.Count - 1].Point_step);
        
        float diffMax = 0f;
        for(int i=0; i<points.Length; i++)
        {
            float diff = Mathf.Abs(expectedDistance - points[i].magnitude);
            
            // find max value
            if(diff > diffMax)
            {
                diffMax = diff;
            }
        }

        Assert.That(diffMax, Is.LessThanOrEqualTo(expectedDistance * acceptedError));

        SimulatorROS2Node.RemoveSubscription<sensor_msgs.msg.PointCloud2>(lidarSubscription);

        // callbacks
        void GetMessageCallback(sensor_msgs.msg.PointCloud2 msg)
        {
            lidarMessages.Add(msg);
        }
    }


    /// <summary>
    /// Method for converting RGL QoS to ROS2 QoS
    /// </summary>
    private ROS2.QualityOfServiceProfile ConvertRGLqosToROS2qos(RglQos rglQos)
    {
        ROS2.QualityOfServiceProfile qos = new ROS2.QualityOfServiceProfile();

        switch (rglQos.reliabilityPolicy)
        {
            case RGLUnityPlugin.RGLQosPolicyReliability.QOS_POLICY_RELIABILITY_BEST_EFFORT:
                qos.SetReliability(ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);
                break;
            
            case RGLUnityPlugin.RGLQosPolicyReliability.QOS_POLICY_RELIABILITY_RELIABLE:
                qos.SetReliability(ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE);
                break;

            default:
                qos.SetReliability(ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
                break;
        }

        switch (rglQos.durabilityPolicy)
        {
            case RGLUnityPlugin.RGLQosPolicyDurability.QOS_POLICY_DURABILITY_VOLATILE:
                qos.SetDurability(ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE);
                break;

            case RGLUnityPlugin.RGLQosPolicyDurability.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
                qos.SetDurability(ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
                break;

            default:
                qos.SetDurability(ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
                break;
        }

        switch (rglQos.historyPolicy)
        {
            case RGLUnityPlugin.RGLQosPolicyHistory.QOS_POLICY_HISTORY_KEEP_ALL:
                qos.SetHistory(ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_ALL, rglQos.historyDepth);
                break;

            case RGLUnityPlugin.RGLQosPolicyHistory.QOS_POLICY_HISTORY_KEEP_LAST:
                qos.SetHistory(ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, rglQos.historyDepth);
                break;

            default:
                qos.SetHistory(ROS2.HistoryPolicy.QOS_POLICY_HISTORY_SYSTEM_DEFAULT, rglQos.historyDepth);
                break;
        }

        return qos;
    }

    /// <summary>
    /// Method for converting PointCloud2 data to position in Unity space.
    /// </summary>
    private Vector3[] ConvertCloudPointToUnityPosition(byte[] data, int pointSize)
    {
        int pointsCount = data.Length / pointSize;
        Vector3[] points = new Vector3[pointsCount];

        for(int i=0, k=0; i<pointsCount; i++)
        {
            // point X
            byte[] ptX = new byte[4]
            {
                data[k],
                data[k+1],
                data[k+2],
                data[k+3]
            };
            float x = System.BitConverter.ToSingle(ptX, 0);

            // point Y
            byte[] ptY = new byte[4]
            {
                data[k+4],
                data[k+5],
                data[k+6],
                data[k+7]
            };
            float y = System.BitConverter.ToSingle(ptY, 0);

            // point Z
            byte[] ptZ = new byte[4]
            {
                data[k+8],
                data[k+9],
                data[k+10],
                data[k+11]
            };
            float z = System.BitConverter.ToSingle(ptZ, 0);

            k += pointSize;
  
            points[i] = AWSIM.ROS2Utility.RosToUnityPosition(new Vector3(x,y,z));
        }

        return points;
    }

    /// <summary>
    /// Method for creating PointCloud2 type message subscription.
    /// </summary>
    private void CreatePointCloud2Subscription(PointCloud2Publisher rglRosPublisher)
    {
        QoSSettings qosSettingsLidar = new QoSSettings()
        {
            ReliabilityPolicy = ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        pointCloudMessages = new List<sensor_msgs.msg.PointCloud2>(); // Clear messages
        pointCloudSubscription?.Dispose(); // Dispose previous subscription
        pointCloudSubscription = SimulatorROS2Node.CreateSubscription<sensor_msgs.msg.PointCloud2>(
            rglRosPublisher.topic, msg =>
            {
                pointCloudMessages.Add(msg);
            }, qosSettingsLidar.GetQoSProfile());
    }
}
