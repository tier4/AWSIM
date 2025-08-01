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

namespace Awsim.Usecase.ScenarioSimulatorConnection
{
    // TODO: Allow more flexible sensor configurations to be supported.
    public class ScenarioEgoVehicle : MonoBehaviour, IScenarioEgoVehicle
    {
        [Header("Vehicle")]
        [SerializeField] Transform _vehicleTransform;
        [SerializeField] AccelVehicle _vehicle;
        [SerializeField] AccelVehicleVisualEffect _vehicleVisualEffect;
        [SerializeField] AccelVehicleReportRos2Publisher _vehicleReportRos2Publisher;
        [SerializeField] AccelVehicleControlModeBasedInputter _controlModeBasedVehicleInputter;
        [SerializeField] AccelVehicleRos2Input _ros2VehicleInput;

        [Header("Sensor")]
        [SerializeField] ImuSensor _imuSensor;
        [SerializeField] ImuRos2Publisher _imuRos2Publisher;
        [SerializeField] CameraSensorScheduler _cameraSensorScheduler;
        [SerializeField] CameraSensor _cameraSensor;
        [SerializeField] CameraRos2Publisher _cameraRos2Publisher;
        [SerializeField] GnssSensor _gnssSensor;
        [SerializeField] GnssRos2Publisher _gnssRos2Publisher;

        public void Initialize()
        {
            // Vehicle.
            _vehicle.Initialize();
            _vehicleVisualEffect.Initialize();
            _controlModeBasedVehicleInputter.Initialize();
            _ros2VehicleInput.Initialize();
            _vehicleReportRos2Publisher.Initialize();

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