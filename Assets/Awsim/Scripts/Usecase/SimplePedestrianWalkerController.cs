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
using Awsim.Entity;

namespace Awsim.Usecase
{
    /// <summary>
    /// Pedestrian straight back and forth.
    /// </summary>
    [RequireComponent(typeof(Pedestrian))]
    public class SimplePedestrianWalkerController : MonoBehaviour
    {
        [SerializeField] float _duration;
        [SerializeField] float _speed;
        [SerializeField] Pedestrian _npcPedestrian;
        Vector3 _startPosition;
        Quaternion _startRotation;
        Vector3 _currentPosition;
        Quaternion _currentRotation;

        public void Initialize()
        {
            _startPosition = transform.position;
            _startRotation = transform.rotation;
            _currentPosition = transform.position;
            _currentRotation = transform.rotation;
            _npcPedestrian.Initialize();

            StartCoroutine(Loop());
        }

        public void OnUpdate()
        {
            _npcPedestrian.OnUpdate();
        }

        public void OnFixedUpdate()
        {
            _npcPedestrian.OnFixedUpdate();
        }

        void Reset()
        {
            _npcPedestrian = GetComponent<Pedestrian>();
        }


        IEnumerator Loop()
        {
            while (true)
            {
                yield return MoveForwardRoutine(_duration, _speed);
                yield return RotateRoutine(0.5f, 360f);
                yield return MoveForwardRoutine(_duration, _speed);
                yield return RotateRoutine(0.5f, 360f);
                var npcTransformPos = _npcPedestrian.transform.position;

                // reset
                _npcPedestrian.PoseInput = new Pose(_startPosition, _startRotation);
                _currentPosition = _startPosition;
                _currentRotation = _startRotation;
            }
        }

        IEnumerator MoveForwardRoutine(float duration, float speed)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                yield return new WaitForFixedUpdate();
                _currentPosition += _currentRotation * Vector3.forward * speed * Time.fixedDeltaTime;
                _npcPedestrian.PoseInput = new Pose(_currentPosition, _currentRotation);
            }
        }

        IEnumerator RotateRoutine(float duration, float angularSpeed)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                yield return new WaitForFixedUpdate();
                _currentRotation *= Quaternion.AngleAxis(angularSpeed * Time.fixedDeltaTime, Vector3.up);
                _npcPedestrian.PoseInput = new Pose(_currentPosition, _currentRotation);
            }
        }
    }
}