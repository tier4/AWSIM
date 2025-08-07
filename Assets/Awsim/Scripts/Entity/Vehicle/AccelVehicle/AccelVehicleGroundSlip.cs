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

/// <summary>
/// Controls the slip of the Vehicle in the Collider.
/// </summary>

namespace Awsim.Entity
{
    public class AccelVehicleGroundSlip : MonoBehaviour
    {
        [SerializeField] float _forwardSlipMultiplier;
        [SerializeField] float _sidewaySlipMultiplier;

        void OnTriggerEnter(Collider other)
        {
            var vehicle = other.GetComponentInParent<AccelVehicle>();
            if (vehicle == null)
                return;

            vehicle.ForwardSlipMultiplier = _forwardSlipMultiplier;
            vehicle.SidewaySlipMultiplier = _sidewaySlipMultiplier;
        }

        void OnTriggerExit(Collider other)
        {
            var vehicle = other.GetComponentInParent<AccelVehicle>();
            if (vehicle == null)
                return;

            // TODO: Better to cache the value on OnTriggerEnter and reassign it on OnTriggerExit.
            vehicle.ForwardSlipMultiplier = 1f;
            vehicle.SidewaySlipMultiplier = 1f;
        }
    }
}