using System;
using System.Collections.Generic;
using System.Linq;

namespace AWSIM.Lanelet
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
        public long RefID;

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
        public long ID;
        public Tag[] Tags;
    }

    /// <summary>
    /// Represents an OSM way element.
    /// See: <see href="https://wiki.openstreetmap.org/wiki/Way" />
    /// </summary>
    [Serializable]
    public struct Way
    {
        public long ID;
        public Tag[] Tags;
        public long[] NodeIDs;
    }

    /// <summary>
    /// Represents an OSM relation element.
    /// See: <see href="https://wiki.openstreetmap.org/wiki/Relation" />
    /// </summary>
    [Serializable]
    public struct Relation
    {
        public long ID;
        public Tag[] Tags;
        public RelationMember[] Members;
    }
}
