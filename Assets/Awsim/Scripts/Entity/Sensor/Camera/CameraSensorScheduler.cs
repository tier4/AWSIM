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

using System.Collections;
using UnityEngine;

namespace Awsim.Entity
{
    public class CameraSensorScheduler : MonoBehaviour
    {
        [SerializeField] Component[] _schedulableCameraSensorComponents = null;
        ICameraSensor[] _schedulableCameraSensors = null;

        [SerializeField] int _outputHz = 10;

        public void Initialize()
        {
            // Get ICameraSensor interface from Component.
            _schedulableCameraSensors = new ICameraSensor[_schedulableCameraSensorComponents.Length];
            for (int i = 0; i < _schedulableCameraSensorComponents.Length; i++)
            {
                _schedulableCameraSensors[i] = _schedulableCameraSensorComponents[i].GetComponent<ICameraSensor>();
            }

            // Initialize camera sensor.
            foreach (var e in _schedulableCameraSensors)
                e.Initialize();

            StartCoroutine(SequentialRender());         // TODO: batch rendering.
        }

        IEnumerator SequentialRender()
        {
            while (true)
            {
                for (int i = 0; i < _schedulableCameraSensors.Length; i++)
                {
                    _schedulableCameraSensors[i].DoRender();
                    yield return null;                          // wait for 1 frame
                }

                yield return new WaitForSeconds(1f / _outputHz);
            }
        }
    }
}