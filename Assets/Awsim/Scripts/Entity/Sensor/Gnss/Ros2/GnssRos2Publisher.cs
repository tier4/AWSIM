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

        public void Initialize()
        {
            if (_gnssSensor == null)
            {
                Debug.LogError("[GnssRos2Publisher] GnssSensor not assigned!");
                return;
            }

            _gnssSensor.OnOutput += Publish;

            var qos = _qosSettings.GetQosProfile();
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
        }

        void Publish(GnssSensor.IReadOnlyOutputData data)
        {
            if (_navSatFixPublisher == null || data?.GeoCoordinate == null) return;
            _navSatFixMsg.Latitude = data.GeoCoordinate.Latitude;
            _navSatFixMsg.Longitude = data.GeoCoordinate.Longitude;
            _navSatFixMsg.Altitude  = data.GeoCoordinate.Altitude;

            var headered = _navSatFixMsg as MessageWithHeader;
            if (headered != null)
            {
                AwsimRos2Node.UpdateROSTimestamps(headered);
            }
            _navSatFixPublisher.Publish(_navSatFixMsg);
        }

        void OnDestroy()
        {
            if (_gnssSensor != null)
                _gnssSensor.OnOutput -= Publish;

            if (_navSatFixPublisher != null)
            {
                AwsimRos2Node.RemovePublisher<NavSatFix>(_navSatFixPublisher);
                _navSatFixPublisher = null;
            }
        }
    }
}