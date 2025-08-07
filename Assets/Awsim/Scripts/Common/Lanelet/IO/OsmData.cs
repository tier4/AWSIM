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

using System;

namespace Awsim.Common
{
    /// <summary>
    /// Represents an OSM data. You can load OSM file into <see cref="OsmData"/> and edit its components.
    /// </summary>
    [Serializable]
    public struct OsmData
    {
        public Node[] Nodes;
        public Way[] Ways;
        public Relation[] Relations;

        public static OsmData Read(string path)
        {
            return OsmParser.Read(path);
        }
    }
}
