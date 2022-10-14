using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Convert the data output from ImuSensor to ROS2 msg and Publish.
    /// </summary>
    [RequireComponent(typeof(ImuSensor))]
    public class ImuRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name in Imu msg.
        /// </summary>
        public string topic = "/sensing/imu/tamagawa/imu_raw";

        /// <summary>
        /// Imu sensor frame id.
        /// </summary>
        public string frameId = "tamagawa/imu_link";

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1000,
        };

        IPublisher<sensor_msgs.msg.Imu> imuPublisher;
        sensor_msgs.msg.Imu imuMsg;
        ImuSensor imuSensor;

        // Start is called before the first frame update
        void Start()
        {
            // Get ImuSensor component.
            imuSensor = GetComponent<ImuSensor>();

            // Set callback.
            imuSensor.OnOutputData += Publish;

            // create Imu ros msg.
            imuMsg = new sensor_msgs.msg.Imu()
            {
                Linear_acceleration = new geometry_msgs.msg.Vector3(),
                Angular_velocity = new geometry_msgs.msg.Vector3(),
                Orientation = new geometry_msgs.msg.Quaternion()
                {
                    W = 1,
                    X = 0,
                    Y = 0,
                    Z = 0,
                },
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = frameId,
                }
            };

            // Set covariances to 0.
            for (int i = 0; i < imuMsg.Angular_velocity_covariance.Length; i++)
                imuMsg.Angular_velocity_covariance[i] = 0;
            for (int i = 0; i < imuMsg.Linear_acceleration_covariance.Length; i++)
                imuMsg.Linear_acceleration_covariance[i] = 0;
            for (int i = 0; i < imuMsg.Orientation_covariance.Length; i++)
                imuMsg.Orientation_covariance[i] = 0;

            // Create publisher.
            var qos = qosSettings.GetQoSProfile();
            imuPublisher = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.Imu>(topic, qos);
        }

        void Publish(ImuSensor.OutputData outputData)
        {
            // Convert from Unity to ROS coordinate system and set the value to msg.
            var rosLinearAcceleration = ROS2Utility.UnityToRosPosition(outputData.LinearAcceleration);
            var rosAngularVelocity = ROS2Utility.UnityToRosAngularVelocity(outputData.AngularVelocity);

            // Update msg.
            imuMsg.Linear_acceleration.X = rosLinearAcceleration.x;
            imuMsg.Linear_acceleration.Y = rosLinearAcceleration.y;
            imuMsg.Linear_acceleration.Z = rosLinearAcceleration.z;
            imuMsg.Angular_velocity.X = rosAngularVelocity.x;
            imuMsg.Angular_velocity.Y = rosAngularVelocity.y;
            imuMsg.Angular_velocity.Z = rosAngularVelocity.z;

            // Update msg header.
            var header = imuMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);

            // Publish to ROS2.
            imuPublisher.Publish(imuMsg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<sensor_msgs.msg.Imu>(imuPublisher);
        }
    }
}