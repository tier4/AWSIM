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

using Awsim.Usecase.PcdGeneration;
using Awsim.Common;
using UnityEngine;

namespace Awsim.Scene.PcdGenerationDemo
{
    public class PcdGenerationDemo : MonoBehaviour
    {
        [SerializeField] PcdGenerator _pcdGenerator;
        [SerializeField] FollowCamera _followCamera;


    
        void Start()
        { 
            AwsimRos2Node.Initialize();     // For point cloud time stamp.
            _pcdGenerator.Initialize();
            _followCamera.Initialize();
            _pcdGenerator.OnPcdSaved += HandlePcdSaved;
        }

        void Update()
        {
            if (!_pcdGenerator.enabled)
            {
                return;
            }
            _pcdGenerator.OnUpdate();
            _followCamera.OnUpdate();
        }

        void HandlePcdSaved()
        {
            Debug.Log("Exiting the scene...");
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#endif
        }
    }
}