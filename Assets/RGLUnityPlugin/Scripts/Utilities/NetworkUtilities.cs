// Copyright 2024 Robotec.ai.
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

using System.Net.NetworkInformation;

namespace RGLUnityPlugin
{
    public class NetworkUtilities
    {
        public static bool IsValidIpAddress(in string ip)
        {
            if (ip == "0.0.0.0")
            {
                return true;
            }
            foreach (var nic in NetworkInterface.GetAllNetworkInterfaces())
            {
                foreach (var unicast in nic.GetIPProperties().UnicastAddresses)
                {
                    if (unicast.Address.ToString() == ip)
                    {
                        return true;
                    }
                }
            }
            return false;
        }
    }
}