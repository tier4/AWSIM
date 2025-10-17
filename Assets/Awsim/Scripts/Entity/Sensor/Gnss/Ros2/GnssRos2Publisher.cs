// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;
using ROS2;
using Awsim.Common;
using std_msgs.msg;
using sensor_msgs.msg;

namespace Awsim.Entity
{
    public class GnssRos2Publisher : MonoBehaviour
    {
        [Header("MGRS Output")]
        [SerializeField] string _poseTopic = "/sensing/gnss/pose";
        [SerializeField] string _poseWithCovTopic = "/sensing/gnss/pose_with_covariance";
        [SerializeField] string _mgrsFrame = "/gnss_link";

        [Header("NavSatFix Output")]
        [SerializeField] string _navSatFixTopic = "/sensing/gnss/ublox/nav_sat_fix";
        [SerializeField] string _frameId = "map"; 
        [SerializeField]
        QosSettings _qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                                   DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                                   HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                                   1000);

        [SerializeField] GnssSensor _gnssSensor;

        IPublisher<NavSatFix> _navSatFixPublisher;
        NavSatFix _navSatFixMsg;
        IPublisher<geometry_msgs.msg.PoseStamped> _posePublisher;
        IPublisher<geometry_msgs.msg.PoseWithCovarianceStamped> _poseWithCovarianceStampedPublisher;
        geometry_msgs.msg.PoseStamped _poseMsg;
        geometry_msgs.msg.PoseWithCovarianceStamped _poseWithCovarianceStampedMsg;
        void Reset()
        {
            var instance = GetComponent<GnssSensor>();
            if (instance != null)
            {
                _gnssSensor = instance;
            }
        }
        
        public void Initialize()
        {
            if (_gnssSensor == null)
            {
                Debug.LogError("[GnssRos2Publisher] GnssSensor not assigned!");
                return;
            }

            _gnssSensor.OnOutput += Publish;

            var qos = _qosSettings.GetQosProfile();

            switch (_gnssSensor.OutputMode)
            {
                case GnssOutputMode.NavSatFix:
                    _navSatFixPublisher = AwsimRos2Node.CreatePublisher<NavSatFix>(_navSatFixTopic, qos);
                    _navSatFixMsg = new NavSatFix
                    {
                        Header = new Header { Frame_id = _frameId },
                        Status = new NavSatStatus
                        {
                            Status = NavSatStatus.STATUS_FIX,
                            Service = NavSatStatus.SERVICE_GPS
                        },
                        Position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    };
                    for (int i = 0; i < _navSatFixMsg.Position_covariance.Length; i++)
                        _navSatFixMsg.Position_covariance[i] = 0;
                    break;

                case GnssOutputMode.Mgrs:
                    _posePublisher = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.PoseStamped>(_poseTopic, qos);
                    _poseWithCovarianceStampedPublisher = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(_poseWithCovTopic, qos);

                    _poseMsg = new geometry_msgs.msg.PoseStamped { Header = new Header { Frame_id = _mgrsFrame } };
                    _poseWithCovarianceStampedMsg = new geometry_msgs.msg.PoseWithCovarianceStamped
                    {
                        Header = new std_msgs.msg.Header { Frame_id = _mgrsFrame },
                        Pose = new geometry_msgs.msg.PoseWithCovariance()
                    };
                    for (int i = 0; i < _poseWithCovarianceStampedMsg.Pose.Covariance.Length; i++)
                        _poseWithCovarianceStampedMsg.Pose.Covariance[i] = 0;
                    break;
            }
        }

        void Publish(GnssSensor.IReadOnlyOutputData data)
        {
            switch (_gnssSensor.OutputMode)
            {
                case GnssOutputMode.NavSatFix:
                    if (_navSatFixPublisher == null || data?.GeoCoordinate == null)
                    {
                        return;
                    } 
                    _navSatFixMsg.Latitude = data.GeoCoordinate.Latitude;
                    _navSatFixMsg.Longitude = data.GeoCoordinate.Longitude;
                    _navSatFixMsg.Altitude = data.GeoCoordinate.Altitude;
                    AwsimRos2Node.UpdateROSTimestamps();
                    _navSatFixPublisher.Publish(_navSatFixMsg);
                    break;

                case GnssOutputMode.Mgrs:
                    if (_posePublisher == null || data.Mgrs == null)
                    {
                        return;
                    }
                    var pos = data.Mgrs.Position;
                    _poseMsg.Pose.Position.X = pos.x;
                    _poseMsg.Pose.Position.Y = pos.y;
                    _poseMsg.Pose.Position.Z = pos.z;
                    _poseWithCovarianceStampedMsg.Pose.Pose.Position.X = pos.x;
                    _poseWithCovarianceStampedMsg.Pose.Pose.Position.Y = pos.y;
                    _poseWithCovarianceStampedMsg.Pose.Pose.Position.Z = pos.z;

                    AwsimRos2Node.UpdateROSTimestamps(_poseMsg as MessageWithHeader, _poseWithCovarianceStampedMsg as MessageWithHeader);
                    _posePublisher.Publish(_poseMsg);
                    _poseWithCovarianceStampedPublisher.Publish(_poseWithCovarianceStampedMsg);
                    break;
            }
        }

        void OnDestroy()
        {
            if (_gnssSensor != null)
            {
                _gnssSensor.OnOutput -= Publish;
            }
            AwsimRos2Node.RemovePublisher<NavSatFix>(_navSatFixPublisher);
            AwsimRos2Node.RemovePublisher<geometry_msgs.msg.PoseStamped>(_posePublisher);
            AwsimRos2Node.RemovePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(_poseWithCovarianceStampedPublisher);
        }
    }
}