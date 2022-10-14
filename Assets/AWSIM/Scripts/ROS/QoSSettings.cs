using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// QoS setting class for ROS.
    /// </summary>
    [Serializable]
    public class QoSSettings
    {
        /// <summary>
        /// ROS QoS ReliabilityPolicy.
        /// </summary>
        public ReliabilityPolicy ReliabilityPolicy;

        /// <summary>
        /// ROS QoS DurabilityPolicy.
        /// </summary>
        public DurabilityPolicy DurabilityPolicy;

        /// <summary>
        /// ROS QoS HistoryPolicy.
        /// </summary>
        public HistoryPolicy HistoryPolicy;

        /// <summary>
        /// ROS QoS HistoryPolicy depth.
        /// </summary>
        public int Depth;

        /// <summary>
        /// Get instance of QualityOfServiceProfile.
        /// </summary>
        /// <returns>instance of QualityOfServiceProfile</returns>
        public QualityOfServiceProfile GetQoSProfile()
        {
            var qos = new QualityOfServiceProfile();
            qos.SetReliability(ReliabilityPolicy);
            qos.SetDurability(DurabilityPolicy);
            qos.SetHistory(HistoryPolicy, Depth);
            return qos;
        }
    }
}