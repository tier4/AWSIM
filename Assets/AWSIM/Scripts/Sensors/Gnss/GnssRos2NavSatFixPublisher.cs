using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Convert the data output from GnssSensor to ROS2 NavSatFix msg and Publish.
    /// </summary>
    [RequireComponent(typeof(GnssSensor))]
    public class GnssRos2NavSatFixPublisher : MonoBehaviour
    {
        private enum Status
        {
            STATUS_NO_FIX = -1,
            STATUS_FIX = 0,
            STATUS_SBAS_FIX = 1,
            STATUS_GBAS_FIX = 2,
        }

        private enum Service
        {
            SERVICE_GPS = 1,
            SERVICE_GLONASS = 2,
            SERVICE_COMPASS = 4,
            SERVICE_GALILEO = 8,
        }

        [System.Serializable]
        private struct NavSatStatus
        {
            public Status status;
            public Service service;
        }

        /// <summary>
        /// Topic name in navSatFix msg.
        /// </summary>
        public string navSatFixTopic = "/sensing/gnss/nav_sat_fix";

        /// <summary>
        /// Gnss sensor frame id.
        /// </summary>
        public string frameId = "gnss_link";

        /// <summary>
        /// NavSatStatus
        /// </summary>
        [SerializeField]
        private NavSatStatus navSatStatus = new NavSatStatus()
        {
            status = Status.STATUS_FIX,
            service = Service.SERVICE_GPS,
        };

        /// <summary>
        /// QoS settings.
        /// </summary>
        public QoSSettings qosSettings;

        IPublisher<sensor_msgs.msg.NavSatFix> navSatFixPublisher;
        sensor_msgs.msg.NavSatFix navSatFixMsg;
        GnssSensor gnssSensor;

        void Start()
        {
            // Get GnssSensor component.
            gnssSensor = GetComponent<GnssSensor>();

            // Set callback.
            gnssSensor.OnOutputData += Publish;

            // Create msg.
            navSatFixMsg = new sensor_msgs.msg.NavSatFix()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = frameId,
                },
                Status = new sensor_msgs.msg.NavSatStatus(),
                Latitude = 0,
                Longitude = 0,
                Altitude = 0
            };
            for (int i = 0; i < navSatFixMsg.Position_covariance.Length; i++)
                navSatFixMsg.Position_covariance[i] = 0;

            // Create publisher.
            var qos = qosSettings.GetQoSProfile();
            navSatFixPublisher = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.NavSatFix>(navSatFixTopic, qos);
        }

        void Publish(GnssSensor.OutputData outputData)
        {
            // Update NavSatStatus.
            navSatFixMsg.Status.Status = (sbyte)navSatStatus.status;
            navSatFixMsg.Status.Service = (ushort)navSatStatus.service;

            // Converts data output from GnssSensor to ROS2 msg.
            navSatFixMsg.Latitude = outputData.GeoCoordinate.Latitude;
            navSatFixMsg.Longitude = outputData.GeoCoordinate.Longitude;
            navSatFixMsg.Altitude = outputData.GeoCoordinate.Altitude;

            // Update msg header.
            var navSatFixHeader = navSatFixMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref navSatFixHeader);

            // Publish to ROS2.
            navSatFixPublisher.Publish(navSatFixMsg);
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<sensor_msgs.msg.NavSatFix>(navSatFixPublisher);
        }
    }
}