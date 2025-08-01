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

using System;
using UnityEngine;
using ROS2;

namespace Awsim.Common
{
    /// <summary>
    /// QoS setting class for ROS2.
    /// </summary>
    [Serializable]
    public class QosSettings
    {
        /// <summary>
        /// ROS2 QoS ReliabilityPolicy.
        /// </summary>
        public ReliabilityPolicy ReliabilityPolicy => _reliabilityPolicy;

        /// <summary>
        /// ROS2 QoS DurabilityPolicy.
        /// </summary>
        public DurabilityPolicy DurabilityPolicy => _durabilityPolicy;

        /// <summary>
        /// ROS2 QoS HistoryPolicy.
        /// </summary>
        public HistoryPolicy HistoryPolicy => _historyPolicy;

        /// <summary>
        /// ROS2 QoS HistoryPolicy depth.
        /// </summary>
        public int Depth => _depth;

        [SerializeField] ReliabilityPolicy _reliabilityPolicy;
        [SerializeField] DurabilityPolicy _durabilityPolicy;
        [SerializeField] HistoryPolicy _historyPolicy;
        [SerializeField] int _depth;
        QualityOfServiceProfile qosProfile = null;

        public QosSettings(ReliabilityPolicy reliabilityPolicy, DurabilityPolicy durabilityPolicy, HistoryPolicy historyPolicy, int depth)
        {
            this._reliabilityPolicy = reliabilityPolicy;
            this._durabilityPolicy = durabilityPolicy;
            this._historyPolicy = historyPolicy;
            this._depth = depth;
        }

        /// <summary>
        /// Get instance of QualityOfServiceProfile.
        /// </summary>
        /// <returns>instance of QualityOfServiceProfile</returns>
        public QualityOfServiceProfile GetQosProfile()
        {
            if (qosProfile == null)
                qosProfile = new QualityOfServiceProfile();

            qosProfile.SetReliability(_reliabilityPolicy);
            qosProfile.SetDurability(_durabilityPolicy);
            qosProfile.SetHistory(_historyPolicy, _depth);
            return qosProfile;
        }
    }
}