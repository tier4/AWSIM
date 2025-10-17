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
using Awsim.Common;
using Awsim.UI;
using Awsim.Usecase;
using Awsim.Usecase.AwsimRvizPlugins;
using Awsim.Usecase.TrafficSimulation;
using Awsim.Entity;

namespace Awsim.Scene.AutowareSimulationDemo
{
    public class AutowareSimulationDemo : MonoBehaviour
    {
        [Serializable]
        public class JsonConfiguration
        {
            public float TimeScale;
            public TimeSourceType TimeSourceType;
            public int RandomTrafficSeed;
            public int MaxVehicleCount;
            public AccelVehicleLogitechG29Input.Settings LogitechG29Settings;
            public AccelVehicle.Settings EgoVehicleSettings;
            public EgoPose EgoPose;
        }

        [Serializable]
        public class EgoPose
        {
            public Vector3 Position;
            public Vector3 EulerAngles;
        }

        [Header("UI")]
        [SerializeField] AwsimCanvas _awsimCanvas;
        [SerializeField] AccelVehicleInformationUIWindow _vehicleInfomationUIWindow;
        [SerializeField] CameraSensorUIWindow _cameraSensorUIWIndow;
        [SerializeField] TrafficSettingsUIWindow _trafficSettingsUIWindow;
        [SerializeField] FollowCameraUIWindow _followCameraUIWindow;
        [SerializeField] AccelVehicleMeterUIWindow _vehicleMeterUIWindow;
        [SerializeField] ControlModeUIWindow _controlModeUIWindow;
        [SerializeField] AccelVehicleInputDeviceUIWindow _vehicleInputDeviceUIWindow;

        [Header("Function")]
        [SerializeField] TrafficSimulator _trafficSimulator;
        [SerializeField] SimplePedestrianWalkerController[] _simplePedestrianWalkerControllers;
        [SerializeField] AwsimRvizPluginsClient _awsimRvizPluginsClient;

        [Header("Ego Vehicle")]
        [SerializeField] EgoVehicle _egoVehicle;

        [Header("V2I")]
        [SerializeField] bool _useV2i;
        [SerializeField] V2I _v2i;
        [SerializeField] V2IRos2Publisher _v2iRosPublisher;

        [Header("Common")]
        [SerializeField] ClockRos2Publisher _clockPublisher;
        [SerializeField] string _nodeName = "AWSIM";
        [SerializeField] TimeSourceType _timeSourceType;
        [SerializeField] FollowCamera _followCamera;
#pragma warning disable CS0414 // Remove unused private members warning disabled. Warns when built for not UNITY_EDITOR
        [SerializeField] string _commandLineConfigParam = "--json_path";
#pragma warning restore CS0414

        [Header("Editor debug")]
        [SerializeField] bool _useJsonConfig;
        [Tooltip("Specify this Path if you want to debug Json loading at Unity Editor runtime.")]
        [SerializeField] string _jsonPath;

        void Start()
        {
            // Load json config.
#if !UNITY_EDITOR
            _useJsonConfig = false;
            _jsonPath = string.Empty;

            _useJsonConfig = CommandLineUtility.GetCommandLineArg(out _jsonPath, _commandLineConfigParam);
#endif

            JsonConfiguration jsonConfig = null;

            if (_useJsonConfig)
            {
                jsonConfig = CommandLineUtility.LoadJsonFromPath<JsonConfiguration>(_jsonPath);
                Time.timeScale = jsonConfig.TimeScale;
                ThreadSafeTime.SyncTimeScale();
                _timeSourceType = jsonConfig.TimeSourceType;
            }

            // Initialize common.
            AwsimRos2Node.Initialize(_nodeName, _timeSourceType);
            _clockPublisher.Initialize();
            _followCamera.Initialize();

            _awsimRvizPluginsClient.Initialize();

            // Initialize ego vehicle.
            if (_useJsonConfig)
            {
                var position = jsonConfig.EgoPose.Position - MgrsPosition.Instance.Mgrs.Position;
                var rotation = Quaternion.Euler(jsonConfig.EgoPose.EulerAngles);
                _egoVehicle.Initialize(jsonConfig.LogitechG29Settings, jsonConfig.EgoVehicleSettings, position, rotation);

            }
            else
                _egoVehicle.Initialize();

            // Initialize function.
            foreach (var e in _simplePedestrianWalkerControllers)
                e.Initialize();

            if (_useJsonConfig)
                _trafficSimulator.Initialize(jsonConfig.RandomTrafficSeed, jsonConfig.MaxVehicleCount);
            else
                _trafficSimulator.Initialize();

            if (_useV2i)
            {
                _v2i.Initialize();
                _v2iRosPublisher.Initialize(_v2i);
            }

            // Initialize UI.
            _vehicleInfomationUIWindow.Vehicle = _egoVehicle.Vehicle;
            _vehicleInfomationUIWindow.ControlModeBasedInputter = _egoVehicle.ControlModeBasedInputProvider;
            _cameraSensorUIWIndow.CameraSensor = _egoVehicle.CameraSensor;
            _trafficSettingsUIWindow.TrafficSimulator = _trafficSimulator;
            _followCameraUIWindow.FollowCamera = _followCamera;
            _vehicleMeterUIWindow.Vehicle = _egoVehicle.Vehicle;
            _controlModeUIWindow.ControlModeBasedInputProvider = _egoVehicle.ControlModeBasedInputProvider;
            _vehicleInputDeviceUIWindow.ControlModeBasedInputter = _egoVehicle.ControlModeBasedInputProvider;

            _awsimCanvas.Initialize();
        }

        void Update()
        {
            // Update clock.
            _clockPublisher.OnUpdate();

            // Update traffic.
            _trafficSimulator.OnUpdate();

            _awsimRvizPluginsClient.OnUpdate();

            foreach (var e in _simplePedestrianWalkerControllers)
                e.OnUpdate();

            // Update ego vehicle.
            if (_egoVehicle != null)
                _egoVehicle.OnUpdate();

            // Update Camera.
            _followCamera.OnUpdate();

            // Update UI.
            _awsimCanvas.OnUpdate();
        }

        void FixedUpdate()
        {
            // Fixed update traffic.
            _trafficSimulator.OnFixedUpdate();

            foreach (var e in _simplePedestrianWalkerControllers)
                e.OnFixedUpdate();

            _awsimRvizPluginsClient.OnFixedUpdate();

            // Fixed update ego vehicle.
            if (_egoVehicle != null)
                _egoVehicle.OnFixedUpdate();

            if (_useV2i)
                _v2i.OnFixedUpdate();
        }
    }
}