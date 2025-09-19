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
using System.Collections.Generic;
using System.Linq;

namespace Awsim.Common
{
    public enum OsmElementType
    {
        Node,
        Way,
        Relation
    }

    [Serializable]
    public struct Tag
    {
        public string Key;
        public string Value;
    }

    [Serializable]
    public struct RelationMember
    {
        public string Role;
        public OsmElementType Type;
        public long RefId;

        public string TypeName
        {
            get
            {
                return kTypeNames[this.Type];
            }
            set
            {
                if (!kTypeNames.ContainsValue(value))
                {
                    throw new Exception("Invalid type name.");
                }
                this.Type = kTypeNames
                    .Where(entry => entry.Value == value)
                    .FirstOrDefault().Key;
            }
        }

        private static readonly Dictionary<OsmElementType, string> kTypeNames = new Dictionary<OsmElementType, string>() {
            { OsmElementType.Node, "node" },
            { OsmElementType.Way, "way" },
            { OsmElementType.Relation, "relation" }
        };
    }

    /// <summary>
    /// Represents an OSM node element.
    /// See: <see href="https://wiki.openstreetmap.org/wiki/Node" />
    /// </summary>
    [Serializable]
    public struct Node
    {
        public long Id;
        public Tag[] Tags;
    }

    /// <summary>
    /// Represents an OSM way element.
    /// See: <see href="https://wiki.openstreetmap.org/wiki/Way" />
    /// </summary>
    [Serializable]
    public struct Way
    {
        public long Id;
        public Tag[] Tags;
        public long[] NodeIds;
    }

    /// <summary>
    /// Represents an OSM relation element.
    /// See: <see href="https://wiki.openstreetmap.org/wiki/Relation" />
    /// </summary>
    [Serializable]
    public struct Relation
    {
        public long Id;
        public Tag[] Tags;
        public RelationMember[] Members;
    }
}
