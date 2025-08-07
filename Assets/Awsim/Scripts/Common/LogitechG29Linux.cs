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

using System.Runtime.InteropServices;

namespace Awsim.Common
{
    public static class LogitechG29Linux
    {
        [DllImport("libG29Linux")]
        public static extern bool InitDevice(string deviceName);

        [DllImport("libG29Linux")]
        public static extern void UploadEffect(double toruqe, double attack_length);

        [DllImport("libG29Linux")]
        public static extern double GetPos();
    }
}