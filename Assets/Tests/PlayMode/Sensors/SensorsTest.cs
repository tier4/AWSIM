using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools.Utils;
using AWSIM;
using System;
using System.Linq;

public class SensorsTest
{
    // Scene handling
    AsyncOperation sceneLoader;
    string sceneName = "SensorsTest";
    Scene scene;
    AsyncOperation aOp;

    // Comparers
    Vector3EqualityComparer v3Comparer;

    // Shared settings
    float testDuration = 2.0f;

    // GNSS
    GnssSensor gnssSensor;
    ROS2.ISubscription<geometry_msgs.msg.PoseStamped> gnssPoseSubscription;
    ROS2.ISubscription<geometry_msgs.msg.PoseWithCovarianceStamped> gnssPoseWithCovarianceSubscription;
    List<geometry_msgs.msg.PoseStamped> poseMessages;
    List<geometry_msgs.msg.PoseWithCovarianceStamped> poseWithCovarianceMessages;

    // LiDAR
    RGLUnityPlugin.LidarSensor lidarSensor;

    // Radar
    RGLUnityPlugin.RadarSensor radarSensor;

    // LiDAR & Radar
    ROS2.ISubscription<sensor_msgs.msg.PointCloud2> pointCloudSubscription;
    List<sensor_msgs.msg.PointCloud2> pointCloudMessages;
    RGLUnityPlugin.RGLNodeSequence rglSubgraphYieldOutput;
    const string yieldOutputNodeId = "OUT_YIELD";
    Vector3[] onlyHits = Array.Empty<Vector3>();

    // IMU
    ImuSensor imuSensor;
    ROS2.ISubscription<sensor_msgs.msg.Imu> imuSubscription;
    List<sensor_msgs.msg.Imu> imuMessages;

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
        radarSensor = GameObject.FindObjectOfType<RGLUnityPlugin.RadarSensor>();
        imuSensor = GameObject.FindObjectOfType<ImuSensor>();

        LidarOutputRestrictionTestSetup();

        yield return null;
    }

    public void LidarOutputRestrictionTestSetup()
    {
        rglSubgraphYieldOutput = new RGLUnityPlugin.RGLNodeSequence()
            .AddNodePointsYield(yieldOutputNodeId, RGLUnityPlugin.RGLField.XYZ_VEC3_F32);

        rglSubgraphYieldOutput.SetPriority(yieldOutputNodeId, 1);
        rglSubgraphYieldOutput.SetActive(yieldOutputNodeId, true);

        lidarSensor.ConnectToWorldFrame(rglSubgraphYieldOutput);
        lidarSensor.onNewData += OnNewLidarData;
    }

    [UnityTest]
    public IEnumerator LiDAR()
    {
        Assert.NotNull(lidarSensor);
        RglLidarPublisher lidarRos2Publisher = lidarSensor.GetComponent<RglLidarPublisher>();

        Assert.AreEqual((byte)lidarRos2Publisher.qos.reliabilityPolicy, (byte)ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);

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

    [UnityTest]
    public IEnumerator Radar()
    {
        Assert.NotNull(radarSensor);
        RglLidarPublisher radarRos2Publisher = radarSensor.GetComponent<RglLidarPublisher>();

        Assert.AreEqual((byte)radarRos2Publisher.qos.reliabilityPolicy, (byte)ROS2.ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);

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
            var dataVec = new Vector3(
                (float)imu.Linear_acceleration.X,
                (float)imu.Linear_acceleration.Y,
                (float)imu.Linear_acceleration.Z
            );
            Assert.That(dataVec, Is.EqualTo(Vector3.zero).Using(v3Comparer));
        });
    }


    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        EditorSceneManager.UnloadScene(scene);
    }

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

    private void OnNewLidarData()
    {
        rglSubgraphYieldOutput.GetResultData(ref onlyHits);
        float startingAzimuth = lidarSensor.outputRestriction.rectangularRestrictionMasks[0].startingHorizontalAngle;
        float endingAzimuth = lidarSensor.outputRestriction.rectangularRestrictionMasks[0].endingHorizontalAngle;
        float startingElevation = lidarSensor.outputRestriction.rectangularRestrictionMasks[0].startingVerticalAngle;
        float endingElevation = lidarSensor.outputRestriction.rectangularRestrictionMasks[0].endingVerticalAngle;

        foreach (var point in onlyHits)
        {
            Vector3 toHitVector = point - lidarSensor.transform.position;
            Vector3 xzProjected = Vector3.ProjectOnPlane(toHitVector, Vector3.up);
            float azimuth = Mathf.Atan2(xzProjected.x, xzProjected.z) * Mathf.Rad2Deg;
            Vector3 xyProjected = Vector3.ProjectOnPlane(toHitVector, Vector3.right);
            float elevation = Mathf.Atan2(xyProjected.y, xyProjected.z) * Mathf.Rad2Deg;

            Assert.IsFalse(azimuth > startingAzimuth && azimuth < endingAzimuth && elevation > startingElevation && elevation < endingElevation);
        }
    }
}