using System.Linq;
using UnityEngine;

namespace AWSIM.Lanelet
{
    public class OsmToLaneletMap
    {
        private Vector3 origin;
        private LaneletMap map;

        public OsmToLaneletMap(Vector3 origin)
        {
            this.origin = origin;
        }

        public LaneletMap Convert(OsmData osm)
        {
            map = new LaneletMap();
            LoadNodes(osm);
            LoadWays(osm);
            LoadLanelets(osm);
            LoadRegulatoryElements(osm);
            return map;
        }

        private void LoadNodes(OsmData osm)
        {
            foreach (var node in osm.Nodes)
            {
                var attributes = new Attributes(node.Tags);
                attributes.TryGetLocalPosition(out Vector3 localPosition);
                var position = ROS2Utility.RosToUnityPosition(localPosition - origin);
                map.Add(new PointData(node.ID, attributes, position));
            }
        }

        private void LoadWays(OsmData osm)
        {
            foreach (var way in osm.Ways)
            {
                var attributes = new Attributes(way.Tags);
                var result = attributes.TryGetValue(AttributeKeys.Area, out bool isArea);
                if (result && isArea)
                {
                    continue;
                }
                var lineString = ExtractLineStringData(way);
                map.Add(lineString);
            }
        }

        private void LoadLanelets(OsmData osm)
        {
            foreach (var relation in osm.Relations)
            {
                var attributes = new Attributes(relation.Tags);
                if (attributes[AttributeKeys.Type] != AttributeValues.Lanelet)
                {
                    continue;
                }
                var id = relation.ID;
                var leftBorder = GetLine(relation, "left");
                var rightBorder = GetLine(relation, "right");
                var lanelet = new LaneletData(id, attributes, leftBorder, rightBorder);
                map.Add(lanelet);
            }
        }

        private void LoadRegulatoryElements(OsmData osm)
        {
            foreach (var relation in osm.Relations)
            {
                var attributes = new Attributes(relation.Tags);
                if (attributes[AttributeKeys.Type] != AttributeValues.RegulatoryElement)
                    continue;

                var id = relation.ID;
                var subtype = attributes[AttributeKeys.Subtype];

                switch (subtype)
                {
                    case AttributeValues.RightOfWay:
                        var yields = GetLanelets(relation, "yield");
                        var rightOfWays = GetLanelets(relation, "right_of_way");
                        var regElem = RegulatoryElement.CreateRightOfWay(
                            id, attributes, yields, rightOfWays);
                        map.Add(regElem);
                        break;
                    case AttributeValues.TrafficSign:
                        var refLines = GetLines(relation, "ref_line");
                        var refers = GetLines(relation, "refers");
                        regElem = RegulatoryElement.CreateTrafficSign(
                            id, attributes, refLines, refers);
                        map.Add(regElem);
                        break;
                    case AttributeValues.TrafficLight:
                        refLines = GetLines(relation, "ref_line");
                        refers = GetLines(relation, "refers");
                        var lightBulbs = GetLines(relation, "light_bulbs");
                        regElem = RegulatoryElement.CreateTrafficLight(
                            id, attributes, refLines, refers, lightBulbs);
                        map.Add(regElem);
                        break;
                }
            }

            foreach (var relation in osm.Relations)
            {
                var attributes = new Attributes(relation.Tags);
                if (attributes[AttributeKeys.Type] != AttributeValues.Lanelet)
                    continue;

                var id = relation.ID;
                var regulatoryElements = GetRegulatoryElements(relation);
                map.Lanelets[id].RegulatoryElements = regulatoryElements;
            }
        }

        private LineStringData ExtractLineStringData(Way way)
        {
            return new LineStringData(way.ID, new Attributes(way.Tags), way.NodeIDs.Select(id => map.Points[id]).ToArray());
        }

        private LineStringData GetLine(Relation relation, string role)
        {
            var id = relation.Members.FirstOrDefault(member => member.Role == role).RefID;
            if (id == 0)
            {
                return null;
            }
            return map.Lines[id];
        }

        private LineStringData[] GetLines(Relation relation, string role)
        {
            return relation.Members
                .Where(member => member.Role == role)
                .Select(member => map.Lines[member.RefID])
                .ToArray();
        }

        private RegulatoryElement[] GetRegulatoryElements(Relation relation)
        {
            return relation.Members
                .Where(member => member.Role == "regulatory_element")
                .Where(member => map.RegulatoryElements.ContainsKey(member.RefID))
                .Select(member => map.RegulatoryElements[member.RefID])
                .ToArray();
        }

        private LaneletData[] GetLanelets(Relation relation, string role)
        {
            return relation.Members
                .Where(member => member.Role == role)
                .Select(member => map.Lanelets[member.RefID])
                .ToArray();
        }
    }
}
