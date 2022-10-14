using System;

namespace AWSIM.Lanelet
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
