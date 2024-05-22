using System.Collections.Generic;

namespace AWSIM.Lanelet
{
    /// <summary>
    /// A component of <see cref="LaneletMap"/>.
    /// </summary>
    /// <typeparam name="TPrimitive"></typeparam>
    public class PrimitiveLayer<TPrimitive> : Dictionary<long, TPrimitive> where TPrimitive : PrimitiveData
    {
        public bool Exists(long id)
        {
            return ContainsKey(id);
        }
    }

    /// <summary>
    /// Represent a basic storage container for lanelet primitives.
    /// See <see href="https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/5001af17ed5bc548edce28eed05a99e6e072e691/lanelet2_core/doc/Architecture.md">Architecture.md</see> for data architecture of Lanelet2.
    /// </summary>
    public class LaneletMap
    {
        public PrimitiveLayer<PointData> Points { get; private set; } = new PrimitiveLayer<PointData>();
        public PrimitiveLayer<LineStringData> Lines { get; private set; } = new PrimitiveLayer<LineStringData>();
        public PrimitiveLayer<LaneletData> Lanelets { get; private set; } = new PrimitiveLayer<LaneletData>();
        public PrimitiveLayer<RegulatoryElement> RegulatoryElements { get; private set; } = new PrimitiveLayer<RegulatoryElement>();


        /// <summary>
        /// Add lanelet if the same ID does not exist.
        /// </summary>
        /// <param name="lanelet"></param>
        public void Add(LaneletData lanelet)
        {
            if (this.Lanelets.Exists(lanelet.ID))
            {
                return;
            }
            Add(lanelet.LeftBound);
            Add(lanelet.RightBound);
            this.Lanelets.Add(lanelet.ID, lanelet);
        }

        /// <summary>
        /// Add line if the same ID does not exist.
        /// </summary>
        /// <param name="lanelet"></param>
        public void Add(LineStringData line)
        {
            if (this.Lines.Exists(line.ID))
            {
                return;
            }
            foreach (var point in line.Points)
            {
                Add(point);
            }
            this.Lines.Add(line.ID, line);
        }

        /// <summary>
        /// Add point if the same ID does not exist.
        /// </summary>
        /// <param name="lanelet"></param>
        public void Add(PointData point)
        {
            if (this.Points.Exists(point.ID))
            {
                return;
            }
            this.Points.Add(point.ID, point);
        }

        /// <summary>
        /// Add regulatory element if the same ID does not exist.
        /// </summary>
        /// <param name="regElem"></param>
        public void Add(RegulatoryElement regElem)
        {
            if (this.RegulatoryElements.Exists(regElem.ID))
            {
                return;
            }
            this.RegulatoryElements.Add(regElem.ID, regElem);
        }
    }
}
