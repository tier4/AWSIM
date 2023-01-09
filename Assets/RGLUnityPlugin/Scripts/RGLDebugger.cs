// Copyright 2022 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;

namespace RGLUnityPlugin
{
    public class RGLDebugger : MonoBehaviour
    {
        [field: SerializeField, Tooltip("Logging verbosity level")]
        public RGLLogLevel LogLevel { get; private set; } = RGLLogLevel.OFF;

        [field: SerializeField, Tooltip("Path to the file where logs will be saved")]
        public string LogOutputPath { get; private set; } = "";

        [field: SerializeField, Tooltip("Path to the file where tape recording will be saved (should contain filename without extension)")]
        public string TapeOutputPath { get; private set; } = "";

        [field: SerializeField, Tooltip("Tape recording activation button")]
        public bool TapeRecord { get; private set; } = false;

        private bool TapeRecordPrev = false;
        private bool isStarted = false;

        public void Start()
        {
            isStarted = true;
        }

        public void OnValidate()
        {
            if (!isStarted)
            {
                TapeRecordPrev = TapeRecord;
                return;
            }

            if (TapeRecord != TapeRecordPrev)
            {
                if (TapeRecord)
                {
                    RGLNativeAPI.TapeRecordBegin(TapeOutputPath);
                }
                else
                {
                    RGLNativeAPI.TapeRecordEnd();
                }
            }
            TapeRecordPrev = TapeRecord;
        }

        void OnDisable()
        {
            if (TapeRecord)
            {
                RGLNativeAPI.TapeRecordEnd();
            }
            TapeRecord = false;
            TapeRecordPrev = false;
        }
    }
}
