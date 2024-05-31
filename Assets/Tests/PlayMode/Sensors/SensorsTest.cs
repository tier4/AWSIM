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

    // IMU
    List<sensor_msgs.msg.Imu> imuMessages;



     // --- TEST LIFE CYCLE ---//

    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        // Scene async operation
        aOp = EditorSceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);

        // Comparers
        v3Comparer = new Vector3EqualityComparer(10e-6f);

        // GNSS
        poseMessages = new List<geometry_msgs.msg.PoseStamped>();
        poseWithCovarianceMessages = new List<geometry_msgs.msg.PoseWithCovarianceStamped>();

        //LiDAR
        lidarMessages = new List<sensor_msgs.msg.PointCloud2>();

        //IMU
        imuMessages = new List<sensor_msgs.msg.Imu>();

    }

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

        yield return null;
    }

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

    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        EditorSceneManager.UnloadScene(scene);
    }


    // --- TEST ROUTINES --- //

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

    [UnityTest]
    public IEnumerator IMU()
    {
        string testScenario = "Imu";
        yield return SetupEnvironment(testScenario);
        yield return new WaitForFixedUpdate();

        ImuSensor imuSensor = GameObject.FindObjectOfType<ImuSensor>();
        Assert.NotNull(imuSensor);
        ImuRos2Publisher ImuRos2Publisher = imuSensor.GetComponent<ImuRos2Publisher>();

        ROS2.ISubscription<sensor_msgs.msg.Imu> imuSubscription = SimulatorROS2Node.CreateSubscription<sensor_msgs.msg.Imu>(
            ImuRos2Publisher.topic, msg =>
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

}
