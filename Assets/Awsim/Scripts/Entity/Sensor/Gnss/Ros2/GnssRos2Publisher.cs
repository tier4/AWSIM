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
using autoware_sensing_msgs.msg;
using System.Threading;
using System.Collections.Generic;
using System.Linq; 
using Unity.Mathematics;

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

        [Header("Orientation Output")]
        [SerializeField] string _orientationTopic = "/sensing/gnss/orientation";
        [SerializeField]
        QosSettings _qosSettings = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                                   DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                                   HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
                                                   1000);

        [SerializeField] GnssSensor _gnssSensor;

        struct GnssDatas
        {
            public int PublishSec;
            public uint PublishNanoSec;
            public int DataSec;
            public uint DataNanoSec;
            public geometry_msgs.msg.PoseStamped Pose;
            public geometry_msgs.msg.PoseWithCovarianceStamped PoseWithCovariance;
            public NavSatFix NavSatFix;
            public GnssInsOrientationStamped Orientation;
        };

        GnssDatas _tmpData;
        bool _dataReady = false;

        [Header("Thread for high frequency update")]
        [SerializeField] volatile int _highFreqUpdateHz = 1000;
        private volatile bool _stopHighFreqUpdate = false;
        private Thread _highFreqUpdateThread;

        //mesurement delay variables
        private volatile float _totalMeasuredDelayMs = 0.0f; // 設定した遅延+システムの遅延
        private volatile float _totalMeasuredDelayMsMin = 1e6f; // 設定した遅延+システムの遅延
        private volatile float _publishMeasuredDelayMs = 0.0f; // 設定した遅延を抜いたシステムの遅延
        private volatile float _publishMeasuredDelayMsMax = 0.0f; // 設定した遅延を抜いたシステムの遅延

        private int counter = 0;

        // Random generator for delay
        [Header("Publish Delay Settings")]
        [SerializeField] bool _gammaDelay = false;
        [SerializeField] float _gammaDelayMeanMs = 0;
        [SerializeField] float _gammaDelayVariance = 0;
        [SerializeField] float _gammaDelayMinMs = 0;
        [SerializeField] float _gammaDelayMaxMs = 10000;

        int _pqmsg_size = 0;

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

            // Initialize temporary MGRS message
            _tmpData.Pose = new geometry_msgs.msg.PoseStamped
            {
                Header = new Header { Frame_id = _mgrsFrame }
            };
            _tmpData.PoseWithCovariance = new geometry_msgs.msg.PoseWithCovarianceStamped
            {
                Header = new Header { Frame_id = _mgrsFrame },
                Pose = new geometry_msgs.msg.PoseWithCovariance()
            };
            for (int i = 0; i < _tmpData.PoseWithCovariance.Pose.Covariance.Length; i++)
                        _tmpData.PoseWithCovariance.Pose.Covariance[i] = 0;

            // Initialize temporary NavSatFix message
             _tmpData.NavSatFix = new NavSatFix
            {
                Header = new Header { Frame_id = _frameId },
                Status = new NavSatStatus
                {
                    Status = NavSatStatus.STATUS_FIX,
                    Service = NavSatStatus.SERVICE_GPS
                },

                Position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            };

            for (int i = 0; i < _tmpData.NavSatFix.Position_covariance.Length; i++)
                _tmpData.NavSatFix.Position_covariance[i] = 0;

            // Initialize temporary Orientation message
            _tmpData.Orientation = new GnssInsOrientationStamped
            {
                Header = new Header { Frame_id = _frameId },
                Orientation = new GnssInsOrientation
                {
                    Orientation = new geometry_msgs.msg.Quaternion()
                }
            };

            //fix gamma parameters
            if (_gammaDelayMeanMs < _gammaDelayMinMs)
            {
                _gammaDelayMeanMs = _gammaDelayMinMs;
            }

            _gnssSensor.OnOutput += DataUpdate;
            _highFreqUpdateThread = new Thread(HighFrequencyUpdate);
            _highFreqUpdateThread.Start();
            
            // publisher
        }

        void DataUpdate(GnssSensor.IReadOnlyOutputData data)
        {
            if (data?.GeoCoordinate == null || data.AttEuler == null)
            {
                return;
            }
            if (_dataReady == true)
            {
                // Exclusive Data Access
                Debug.LogWarning("[GnssRos2Publisher] Previous data has not been loaded yet.");
                return;
            }

            //Data copy
            //MRGS
            var pos = data.Mgrs.Position;
            _tmpData.Pose.Pose.Position.X = pos.x;
            _tmpData.Pose.Pose.Position.Y = pos.y;
            _tmpData.Pose.Pose.Position.Z = pos.z;
            _tmpData.PoseWithCovariance.Pose.Pose.Position.X = pos.x;
            _tmpData.PoseWithCovariance.Pose.Pose.Position.Y = pos.y;
            _tmpData.PoseWithCovariance.Pose.Pose.Position.Z = pos.z;

            //NavSatFix
            _tmpData.NavSatFix.Latitude = data.GeoCoordinate.Latitude;
            _tmpData.NavSatFix.Longitude = data.GeoCoordinate.Longitude;
            _tmpData.NavSatFix.Altitude = data.GeoCoordinate.Altitude;
            
            // septentrio heading converter
            // https://github.com/tier4/aip_launcher/blob/tier4/universe/aip_common_sensor_launch/scripts/septentrio_heading_converter.py
            _tmpData.Orientation.Orientation.Orientation.X = 0.0;
            _tmpData.Orientation.Orientation.Orientation.Y = 0.0;
            _tmpData.Orientation.Orientation.Orientation.Z = math.sin(math.radians(90 - data.AttEuler.heading) / 2.0);
            _tmpData.Orientation.Orientation.Orientation.W = math.cos(math.radians(90 - data.AttEuler.heading) / 2.0);

            // This assumes the covariance is 1.0.
            _tmpData.Orientation.Orientation.Rmse_rotation_x = 1.0f;
            _tmpData.Orientation.Orientation.Rmse_rotation_y = 1.0f;
            _tmpData.Orientation.Orientation.Rmse_rotation_z = 1.0f;

            //タイムスタンプ押下
            AwsimRos2Node.UpdateROSTimestamps(_tmpData.Pose as MessageWithHeader, _tmpData.PoseWithCovariance as MessageWithHeader,_tmpData.NavSatFix as MessageWithHeader, _tmpData.Orientation as MessageWithHeader);
            
            //Debug logs for delay measurement
            //Debug.Log($"[GnssRos2Publisher] Total Delay: {_totalMeasuredDelayMs} ms");
            //Debug.Log($"[GnssRos2Publisher] Total Delay Min: {_totalMeasuredDelayMsMin} ms");
            //Debug.Log($"[GnssRos2Publisher] Publish Delay: {_publishMeasuredDelayMs} ms");
            //Debug.Log($"[GnssRos2Publisher] Publish Delay Max: {_publishMeasuredDelayMsMax} ms");

            //Setting Delay
            double delay_sec = 0.0f;
            if (_gammaDelay)
            {
                float delay_ms = _gammaDelayMinMs + GenerateGamma(_gammaDelayMeanMs - _gammaDelayMinMs, _gammaDelayVariance);
                delay_ms = Mathf.Clamp(delay_ms, _gammaDelayMinMs, _gammaDelayMaxMs);
                delay_sec = (double)(delay_ms / 1000.0f);
            }
            // Debug.Log($"[GnssRos2Publisher] Delay Sec: {delay_sec} s");

            int now_sec = _tmpData.NavSatFix.Header.Stamp.Sec;
            uint now_nanosec = _tmpData.NavSatFix.Header.Stamp.Nanosec; // time stamp

            // Debug.Log($"[GnssRos2Publisher] NowSec: {now_sec} s");

            _tmpData.DataSec = now_sec;
            _tmpData.DataNanoSec = now_nanosec;
            _tmpData.PublishSec = now_sec + (int)((now_nanosec + (uint)(delay_sec * 1e9)) / 1_000_000_000);
            _tmpData.PublishNanoSec = (_tmpData.DataNanoSec + (uint)(delay_sec * 1e9)) % 1_000_000_000;

            _dataReady = true;
            //Debug.Log($"[GnssRos2Publisher] pqmsg size: {_pqmsg_size}");
        }



        void HighFrequencyUpdate()
        {
            int indexCounter = 0;//辞書を重複させないためのカウンター
            var pqMsg = new SortedDictionary<(int, uint, int), GnssDatas>(); // priority queue(SortedDictionaryで代用)
            int period = (int)((1000.0f / _highFreqUpdateHz) + 0.5f);

            var qos = _qosSettings.GetQosProfile();
            IPublisher<geometry_msgs.msg.PoseStamped> posePublisher = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.PoseStamped>(_poseTopic, qos);
            IPublisher<geometry_msgs.msg.PoseWithCovarianceStamped> poseWithCovarianceStampedPublisher = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(_poseWithCovTopic, qos);
            IPublisher<NavSatFix> navSatFixPublisher = AwsimRos2Node.CreatePublisher<NavSatFix>(_navSatFixTopic, qos);
            IPublisher<GnssInsOrientationStamped> orientationPublisher = AwsimRos2Node.CreatePublisher<GnssInsOrientationStamped>(_orientationTopic, qos);

            //準備が整うまで待機

            while (_gnssSensor == null || navSatFixPublisher == null || orientationPublisher == null) // || attEulerPublisher == null)
            {
                Thread.Sleep(period);
            }

            while (!_stopHighFreqUpdate)
            {
                Thread.Sleep(period);
                _pqmsg_size = pqMsg.Count;

                if (_dataReady)
                {
                    // add deep copy to priority queue
                    var newMsg = new GnssDatas
                    {
                        PublishSec = _tmpData.PublishSec,
                        PublishNanoSec = _tmpData.PublishNanoSec,
                        DataSec = _tmpData.DataSec,
                        DataNanoSec = _tmpData.DataNanoSec,
                        Pose = CreateDeepCopyPose(_tmpData.Pose),
                        PoseWithCovariance = CreateDeepCopyPoseWithCovariance(_tmpData.PoseWithCovariance),
                        NavSatFix = CreateDeepCopyNavSatFix(_tmpData.NavSatFix),
                        Orientation = CreateDeepCopyOrientation(_tmpData.Orientation)
                    };

                    pqMsg.Add((newMsg.PublishSec, newMsg.PublishNanoSec, indexCounter++), newMsg);
                    // Exclusive Data Access
                    _dataReady = false;
                }

                //time sourceから時刻を取得(Ros2ClockPublisherによればスレッドセーフなはず)
                AwsimRos2Node.GetTime(out var now_sec, out var now_nanosec);

                // publish ready message
                while ((pqMsg.Count > 0) && ((pqMsg.First().Key.Item1 < now_sec) ||
                       ((pqMsg.First().Key.Item1 == now_sec) && (pqMsg.First().Key.Item2 <= now_nanosec))))
                {
                    var first_item = pqMsg.First();
                    posePublisher.Publish(first_item.Value.Pose);
                    poseWithCovarianceStampedPublisher.Publish(first_item.Value.PoseWithCovariance);
                    navSatFixPublisher.Publish(first_item.Value.NavSatFix);
                    orientationPublisher.Publish(first_item.Value.Orientation);

                    _totalMeasuredDelayMs = (float)(now_sec - first_item.Value.DataSec) * 1000.0f +
                                            (float)(now_nanosec - first_item.Value.DataNanoSec) / 1e6f;

                    _publishMeasuredDelayMs =
                        (now_sec - first_item.Value.PublishSec) * 1000.0f +
                    (now_nanosec - first_item.Value.PublishNanoSec) / 1e6f;

                    _totalMeasuredDelayMsMin = Mathf.Min(_totalMeasuredDelayMsMin, _totalMeasuredDelayMs);
                    _publishMeasuredDelayMsMax = Mathf.Max(_publishMeasuredDelayMsMax, _publishMeasuredDelayMs);

                    pqMsg.Remove(first_item.Key);
                }
            }

            // OnDestroy
            AwsimRos2Node.RemovePublisher<geometry_msgs.msg.PoseStamped>(posePublisher);
            AwsimRos2Node.RemovePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(poseWithCovarianceStampedPublisher);
            AwsimRos2Node.RemovePublisher<NavSatFix>(navSatFixPublisher);
            AwsimRos2Node.RemovePublisher<GnssInsOrientationStamped>(orientationPublisher);
        }

        private static geometry_msgs.msg.PoseStamped CreateDeepCopyPose(geometry_msgs.msg.PoseStamped original)
        {
            var newPose = new geometry_msgs.msg.PoseStamped
            {
                Header = new Header
                {
                    Stamp = new builtin_interfaces.msg.Time
                    {
                        Sec = original.Header.Stamp.Sec,
                        Nanosec = original.Header.Stamp.Nanosec
                    },
                    Frame_id = original.Header.Frame_id
                },

                Pose = new geometry_msgs.msg.Pose
                {
                    Position = new geometry_msgs.msg.Point
                    {
                        X = original.Pose.Position.X,
                        Y = original.Pose.Position.Y,
                        Z = original.Pose.Position.Z
                    },

                    Orientation = new geometry_msgs.msg.Quaternion
                    {
                        X = original.Pose.Orientation.X,
                        Y = original.Pose.Orientation.Y,
                        Z = original.Pose.Orientation.Z,
                        W = original.Pose.Orientation.W
                    }
                }
            };
            return newPose;
        }
        
        private static geometry_msgs.msg.PoseWithCovarianceStamped CreateDeepCopyPoseWithCovariance(geometry_msgs.msg.PoseWithCovarianceStamped original)
        {
            var newPoseWithCov = new geometry_msgs.msg.PoseWithCovarianceStamped
            {
                Header = new Header
                {
                    Stamp = new builtin_interfaces.msg.Time
                    {
                        Sec = original.Header.Stamp.Sec,
                        Nanosec = original.Header.Stamp.Nanosec
                    },
                    Frame_id = original.Header.Frame_id
                },

                Pose = new geometry_msgs.msg.PoseWithCovariance
                {
                    Pose = new geometry_msgs.msg.Pose
                    {
                        Position = new geometry_msgs.msg.Point
                        {
                            X = original.Pose.Pose.Position.X,
                            Y = original.Pose.Pose.Position.Y,
                            Z = original.Pose.Pose.Position.Z
                        },

                        Orientation = new geometry_msgs.msg.Quaternion
                        {
                            X = original.Pose.Pose.Orientation.X,
                            Y = original.Pose.Pose.Orientation.Y,
                            Z = original.Pose.Pose.Orientation.Z,
                            W = original.Pose.Pose.Orientation.W
                        }
                    }
                }
            };

            for (int i = 0; i < original.Pose.Covariance.Length; i++)
                newPoseWithCov.Pose.Covariance[i] = original.Pose.Covariance[i];

            return newPoseWithCov;
        }




        private static NavSatFix CreateDeepCopyNavSatFix(NavSatFix original)
        {
            var newNavSatFix = new NavSatFix
            {
                Header = new Header
                {
                    Stamp = new builtin_interfaces.msg.Time
                    {
                        Sec = original.Header.Stamp.Sec,
                        Nanosec = original.Header.Stamp.Nanosec
                    },
                    Frame_id = original.Header.Frame_id
                },
                Status = new NavSatStatus
                {
                    Status = original.Status.Status,
                    Service = original.Status.Service
                },
                Latitude = original.Latitude,
                Longitude = original.Longitude,
                Altitude = original.Altitude,
                Position_covariance_type = original.Position_covariance_type
            };
            for (int i = 0; i < original.Position_covariance.Length; i++)
                newNavSatFix.Position_covariance[i] = original.Position_covariance[i];

            return newNavSatFix;
        }

        private static GnssInsOrientationStamped CreateDeepCopyOrientation(GnssInsOrientationStamped original)
        {
            var newOrientation = new GnssInsOrientationStamped
            {
                Header = new Header
                {
                    Stamp = new builtin_interfaces.msg.Time
                    {
                        Sec = original.Header.Stamp.Sec,
                        Nanosec = original.Header.Stamp.Nanosec
                    },
                    Frame_id = original.Header.Frame_id
                },
                Orientation = new GnssInsOrientation
                {
                    Orientation = new geometry_msgs.msg.Quaternion
                    {
                        X = original.Orientation.Orientation.X,
                        Y = original.Orientation.Orientation.Y,
                        Z = original.Orientation.Orientation.Z,
                        W = original.Orientation.Orientation.W
                    },
                    Rmse_rotation_x = original.Orientation.Rmse_rotation_x,
                    Rmse_rotation_y = original.Orientation.Rmse_rotation_y,
                    Rmse_rotation_z = original.Orientation.Rmse_rotation_z
                }
            };

            return newOrientation;
        }

        void OnDestroy()
        {
            _stopHighFreqUpdate = true;
            if (_gnssSensor != null)
            {
                _gnssSensor.OnOutput -= DataUpdate;
            }
        }

        public static float GenerateGamma(float mean, float variance)
        {
            // https://github.com/mathnet/mathnet-numerics/blob/master/src/Numerics/Distributions/Gamma.cs
            if (variance <= 0.0)
            {
                return mean;
            }
            //gamma distribution parameters
            float rate = mean / variance; // 1/\theta
            float shape = mean * mean / variance; // k
            var a = shape;
            var alphafix = 1.0f;

            // Fix when alpha is less than one.
            if (shape < 1.0)
            {
                a = shape + 1.0f;
                alphafix = (float)System.Math.Pow(UnityEngine.Random.Range(0f, 1f), 1.0 / shape);
            }

            var d = a - (1.0f / 3.0f);
            var c = 1.0f / (float)System.Math.Sqrt(9.0 * d);
            while (true)
            {
                var x = GenerateGaussian(0.0f, 1.0f);
                var v = 1.0f + (c * x);
                while (v <= 0.0)
                {
                    x = GenerateGaussian(0.0f, 1.0f);
                    v = 1.0f + (c * x);
                }
                v = v * v * v;
                var u = UnityEngine.Random.Range(0f, 1f);
                x = x * x;
                if (u < 1.0f - (0.0331f * x * x))
                {
                    return alphafix * d * v / rate;
                }

                if (System.Math.Log(u) < (0.5f * x) + (d * (1.0f - v + System.Math.Log(v))))
                {
                    return alphafix * d * v / rate;
                }
            }
        }

        private static float GenerateGaussian(float mean, float stdDev)
        {
            // Generate Gaussian (normal) noise using Box-Muller transform
            float u1 = 1.0f - UnityEngine.Random.Range(0f, 1f); // avoid 0
            float u2 = 1.0f - UnityEngine.Random.Range(0f, 1f);
            float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
            return randStdNormal * stdDev + mean;
        }
    }
}