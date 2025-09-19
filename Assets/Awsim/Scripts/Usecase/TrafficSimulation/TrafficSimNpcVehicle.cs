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

namespace Awsim.Usecase.TrafficSimulation
{
    public class TrafficSimNpcVehicle : MonoBehaviour
    {
        public PoseVehicle NpcVehicle => _npcVehicle;
        public uint Id => _id;

        [SerializeField] PoseVehicle _npcVehicle;
        uint _id;

        public void Initialize(uint id)
        {
            _id = id;
            _npcVehicle.Initialize();
        }

        public void OnUpdate()
        {
            NpcVehicle.OnUpdate();
        }

        public void OnFixedUpdate()
        {
            NpcVehicle.OnFixedUpdate();
        }
    }
}