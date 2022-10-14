using System.Linq;
using System.Xml;

namespace AWSIM.Lanelet
{
    /// <summary>
    /// Parser that provides access to <see cref="OsmElement"/> in OSM file.
    /// See: <see href="https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/5001af17ed5bc548edce28eed05a99e6e072e691/lanelet2_io/src/OsmFile.cpp#L176" />
    /// </summary>
    internal static class OsmParser
    {
        internal static OsmData Read(string path)
        {
            var doc = new XmlDocument();
            doc.Load(path);
            var osmXmlNode = doc.SelectSingleNode("osm");
            return new OsmData
            {
                Nodes = ReadNodes(osmXmlNode),
                Ways = ReadWays(osmXmlNode),
                Relations = ReadRelations(osmXmlNode)
            };
        }

        private static long ReadID(XmlNode xmlNode) =>
            long.Parse(xmlNode.Attributes["id"].Value);

        private static Tag[] ReadTags(XmlNode xmlNode) =>
            xmlNode.SelectNodes("tag").Cast<XmlNode>()
            .Select(node => ReadTag(node))
            .ToArray();

        private static Node[] ReadNodes(XmlNode xmlNode) =>
            xmlNode.SelectNodes("node").Cast<XmlNode>()
            .Select(node => ReadNode(node))
            .ToArray();

        private static Way[] ReadWays(XmlNode xmlNode) =>
            xmlNode.SelectNodes("way").Cast<XmlNode>()
            .Select(node => ReadWay(node))
            .ToArray();

        private static Relation[] ReadRelations(XmlNode xmlNode) =>
            xmlNode.SelectNodes("relation").Cast<XmlNode>()
            .Select(node => ReadRelation(node))
            .ToArray();

        private static Tag ReadTag(XmlNode xmlNode) => new Tag()
        {
            Key = xmlNode.Attributes["k"].Value,
            Value = xmlNode.Attributes["v"].Value
        };

        private static Node ReadNode(XmlNode xmlNode) => new Node
        {
            ID = ReadID(xmlNode),
            Tags = ReadTags(xmlNode)
        };

        private static Way ReadWay(XmlNode xmlNode) => new Way
        {
            ID = ReadID(xmlNode),
            Tags = ReadTags(xmlNode),
            NodeIDs = xmlNode.SelectNodes("nd").Cast<XmlNode>()
                .Select(node => long.Parse(node.Attributes["ref"].Value))
                .ToArray()
        };

        private static RelationMember ReadMember(XmlNode xmlNode) => new RelationMember
        {
            RefID = long.Parse(xmlNode.Attributes["ref"].Value),
            Role = xmlNode.Attributes["role"].Value,
            TypeName = xmlNode.Attributes["type"].Value
        };

        private static Relation ReadRelation(XmlNode xmlNode) => new Relation
        {
            ID = ReadID(xmlNode),
            Tags = ReadTags(xmlNode),
            Members = xmlNode.SelectNodes("member").Cast<XmlNode>()
                .Select(node => ReadMember(node)).ToArray()
        };
    }
}
