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
using Awsim.Usecase.ScenarioSimulatorConnection;
using Awsim.Common;

namespace Awsim.Scene.IntegrateScenarioSimulatorDemo
{
    public class IntegrateScenarioSimulatorDemo : MonoBehaviour
    {
        [SerializeField] ScenarioSimulatorClient _scenarioSimulatorClient;
        [SerializeField] ClockRos2Publisher _clockRos2Publisher;

        void Start()
        {
            AwsimRos2Node.Initialize("AWSIM", TimeSourceType.External);
            _clockRos2Publisher.Initialize();
            _scenarioSimulatorClient.Initialize();
        }

        void Update()
        {
            _clockRos2Publisher.OnUpdate();
            _scenarioSimulatorClient.OnUpdate();
        }

        void FixedUpdate()
        {
            _scenarioSimulatorClient.OnFixedUpdate();
        }
    }
}