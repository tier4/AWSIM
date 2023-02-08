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

using System;

namespace RGLUnityPlugin
{
    public class RGLNodeHandle
    {
        /// <summary>
        /// Allows users to reference node for modifications and prevents users from adding the same node twice by accident.
        /// </summary>
        public string Identifier = "";

        /// <summary>
        /// RGL handle to node (rgl_node_t).
        /// </summary>
        public IntPtr Node = IntPtr.Zero;

        /// <summary>
        /// Whether the node is connected to the NodeSequence or not.
        /// </summary>
        public bool Connected = false;

        /// <summary>
        /// Stores output field that can be received.
        /// </summary>
        public RGLField OutputField = RGLField.UNKNOWN;

        /// <summary>
        /// Allows calling correct API call to modify node's arguments.
        /// </summary>
        public RGLNodeType Type = RGLNodeType.UNKNOWN;
    }
}