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
using Awsim.Entity;
using System;
using Awsim.Common;

namespace Awsim.Scene.AutowareSimulationDemo
{
    public class EgoVehicle : MonoBehaviour
    {
        [Serializable]
        public class Settings
        {
            public string LogitechG29DevicePath;
            public EgoPose EgoPose;
            public float FullThrottleAcceleration;  // (m/s^2)
            public float FullBrakeDeceleration;     // (m/s^2)
        }

        [Serializable]
        public class EgoPose
        {
            public Vector3 MgrsPosition;
            public Vector3 EulerAngles;
        }

        public AccelVehicle Vehicle => _vehicle;
        public AccelVehicleControlModeBasedInputter ControlModeBasedInputProvider => _controlModeBasedVehicleInputter;
        public CameraSensor CameraSensor => _cameraSensor;

        [Header("Ego Vehicle")]
        [SerializeField] Transform _vehicleTransform;
        [SerializeField] AccelVehicle _vehicle;
        [SerializeField] AccelVehicleReportRos2Publisher _vehicleReportRos2Publisher;
        [SerializeField] AccelVehicleControlModeSrvServer _controlModeSrvServer;
        [SerializeField] AccelVehicleControlModeBasedInputter _controlModeBasedVehicleInputter;
        [SerializeField] AccelVehicleKeyboardInput _keyboardVehicleInput;
        [SerializeField] AccelVehicleRos2Input _ros2VehicleInput;
        [SerializeField] AccelVehicleLogitechG29Input _logitechG29VehicleInput;
        [SerializeField] AccelVehicleVisualEffect _vehicleVisualEffect;

        [Header("Ego vehicle sensor")]
        [SerializeField] ImuSensor _imuSensor;
        [SerializeField] ImuRos2Publisher _imuRos2Publisher;
        [SerializeField] CameraSensorScheduler _cameraSensorScheduler;
        [SerializeField] CameraSensor _cameraSensor;
        [SerializeField] CameraRos2Publisher _cameraRos2Publisher;
        [SerializeField] GnssSensor _gnssSensor;
        [SerializeField] GnssRos2Publisher _gnssRos2Publisher;

        public void Initialize(Settings settings)
        {
            // Vehicle.
            _vehicle.Initialize();
            _controlModeBasedVehicleInputter.Initialize();
            _controlModeSrvServer.Initialize();
            _vehicleReportRos2Publisher.Initialize();
            _vehicleVisualEffect.Initialize();
            _keyboardVehicleInput.Initialize(settings.FullThrottleAcceleration, settings.FullBrakeDeceleration);
            _ros2VehicleInput.Initialize();
            _logitechG29VehicleInput.Initialize(settings.FullThrottleAcceleration, settings.FullBrakeDeceleration, settings.LogitechG29DevicePath);
            _vehicleTransform.position = settings.EgoPose.MgrsPosition - MgrsPosition.Instance.Mgrs.Position;   // MGRS to Unity.
            _vehicleTransform.rotation = Quaternion.Euler(settings.EgoPose.EulerAngles);

            // Sensor.
            _imuSensor.Initialize();
            _imuRos2Publisher.Initialize();
            _cameraSensorScheduler.Initialize();
            _cameraRos2Publisher.Initialize();
            _gnssSensor.Initialize();
            _gnssRos2Publisher.Initialize();
        }

        public void Initialize()
        {
            // Vehicle.
            _vehicle.Initialize();
            _controlModeBasedVehicleInputter.Initialize();
            _controlModeSrvServer.Initialize();
            _vehicleReportRos2Publisher.Initialize();
            _vehicleVisualEffect.Initialize();
            _keyboardVehicleInput.Initialize();
            _ros2VehicleInput.Initialize();
            _logitechG29VehicleInput.Initialize();

            // Sensor.
            _imuSensor.Initialize();
            _imuRos2Publisher.Initialize();
            _cameraSensorScheduler.Initialize();
            _cameraRos2Publisher.Initialize();
            _gnssSensor.Initialize();
            _gnssRos2Publisher.Initialize();
        }

        public void OnUpdate()
        {
            // Vehicle.
            _controlModeBasedVehicleInputter.OnUpdate();
            _vehicle.OnUpdate();
            _vehicleVisualEffect.OnUpdate();
        }

        public void OnFixedUpdate()
        {
            // Vehicle.
            _controlModeBasedVehicleInputter.OnFixedUpdate();
            _vehicle.OnFixedUpdate();

            // Sensor.
            _imuSensor.OnFixedUpdate();
        }
    }
}