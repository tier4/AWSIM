using NUnit.Framework;
using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using AWSIM;
using System.Linq;

public class SensorsTest
{
    // Scene handling
    AsyncOperation sceneLoader;
    string sceneName = "SensorsTest";
    Scene scene;
    AsyncOperation aOp;

    // Shared settings
    float testDuration = 5.0f;

    // GNSS
    GnssSensor gnssSensor;
    ROS2.ISubscription<geometry_msgs.msg.PoseStamped> gnssPoseSubscription;
    ROS2.ISubscription<geometry_msgs.msg.PoseWithCovarianceStamped> gnssPoseWithCovarianceSubscription;
    List<geometry_msgs.msg.PoseStamped> poseMessages;
    List<geometry_msgs.msg.PoseWithCovarianceStamped> poseWithCovarianceMessages;

    // LiDAR
    RGLUnityPlugin.LidarSensor lidarSensor;
    ROS2.ISubscription<sensor_msgs.msg.PointCloud2> lidarSubscription;
    List<sensor_msgs.msg.PointCloud2> lidarMessages;

    // IMU
    ImuSensor imuSensor;
    ROS2.ISubscription<sensor_msgs.msg.Imu> imuSubscription;
    List<sensor_msgs.msg.Imu> imuMessages;

    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        // Scene async operation
        aOp = EditorSceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);

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

        gnssSensor = GameObject.FindObjectOfType<GnssSensor>();
        lidarSensor = GameObject.FindObjectOfType<RGLUnityPlugin.LidarSensor>();
        imuSensor = GameObject.FindObjectOfType<ImuSensor>();

        yield return null;
    }

    [UnityTest]
    public IEnumerator LiDAR()
    {
        Assert.NotNull(lidarSensor);
        RglLidarPublisher lidarRos2Publisher = lidarSensor.GetComponent<RglLidarPublisher>();
        
        Assert.AreEqual(((byte)lidarRos2Publisher.reliabilityPolicy) , ((byte)ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT));

        QoSSettings QosSettingsLidar = new QoSSettings()
        {
            ReliabilityPolicy = ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
            DurabilityPolicy = ROS2.DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = ROS2.HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };
        
        lidarSubscription = SimulatorROS2Node.CreateSubscription<sensor_msgs.msg.PointCloud2>(
            lidarRos2Publisher.pcl24Topic, msg =>
        {
            lidarMessages.Add(msg);
        }, QosSettingsLidar.GetQoSProfile());

        yield return new WaitForSeconds(testDuration);

        Assert.IsNotEmpty(lidarMessages);

        Assert.AreEqual(lidarMessages.Count, (int)(testDuration * lidarSensor.AutomaticCaptureHz));
    }

    [UnityTest]
    public IEnumerator GNSS()
    {
        Assert.NotNull(gnssSensor);
        GnssRos2Publisher gnssRos2Publisher = gnssSensor.GetComponent<GnssRos2Publisher>();

        gnssPoseSubscription = SimulatorROS2Node.CreateSubscription<geometry_msgs.msg.PoseStamped>(
            gnssRos2Publisher.poseTopic, msg =>
        {
            poseMessages.Add(msg);
        });
        gnssPoseWithCovarianceSubscription = SimulatorROS2Node.CreateSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(
            gnssRos2Publisher.poseWithCovarianceStampedTopic, msg =>
        {
            poseWithCovarianceMessages.Add(msg);
        });

        yield return new WaitForSeconds(testDuration);

        Assert.IsNotEmpty(poseMessages);
        Assert.IsNotEmpty(poseWithCovarianceMessages);

        poseMessages.ForEach(pose =>
        {
            Assert.AreEqual(0.0f, pose.Pose.Position.X);
            Assert.AreEqual(0.0f, pose.Pose.Position.Y);
            Assert.AreEqual(0.0f, pose.Pose.Position.Z);
        });

        poseWithCovarianceMessages.ForEach(pose =>
        {
            Assert.AreEqual(0.0f, pose.Pose.Pose.Position.X);
            Assert.AreEqual(0.0f, pose.Pose.Pose.Position.Y);
            Assert.AreEqual(0.0f, pose.Pose.Pose.Position.Z);
        });

        Assert.AreEqual(poseMessages.Count, (int)(testDuration * gnssSensor.OutputHz));
        Assert.AreEqual(poseWithCovarianceMessages.Count, (int)(testDuration * gnssSensor.OutputHz));

        // TODO: requires R2FU
        // SimulatorROS2Node.RemoveSubscription<geometry_msgs.msg.PoseStamped>(gnssPoseSubscription);
        // SimulatorROS2Node.RemoveSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(gnssPoseWithCovarianceSubscription);
    }

    [UnityTest]
    public IEnumerator IMU()
    {
        Assert.NotNull(imuSensor);
        ImuRos2Publisher ImuRos2Publisher = imuSensor.GetComponent<ImuRos2Publisher>();

        imuSubscription = SimulatorROS2Node.CreateSubscription<sensor_msgs.msg.Imu>(
            ImuRos2Publisher.topic, msg =>
        {
            imuMessages.Add(msg);
        });
        yield return new WaitForSeconds(testDuration);

        Assert.IsNotEmpty(imuMessages);
        Assert.AreEqual(imuMessages.Count, (int)(testDuration * imuSensor.OutputHz));

        imuMessages.ForEach(imu =>
        {
            Assert.AreEqual(0.0f, imu.Linear_acceleration.X);
            Assert.AreEqual(0.0f, imu.Linear_acceleration.Y);
            Assert.AreEqual(0.0f, imu.Linear_acceleration.Z);
        });
    }


    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        EditorSceneManager.UnloadScene(scene);
    }
}
