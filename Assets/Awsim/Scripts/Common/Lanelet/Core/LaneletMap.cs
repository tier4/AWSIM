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

using System.Collections.Generic;

namespace Awsim.Common
{
    /// <summary>
    /// A component of <see cref="LaneletMap"/>.
    /// </summary>
    /// <typeparam name="TPrimitive"></typeparam>
    public class PrimitiveLayer<TPrimitive> : Dictionary<long, TPrimitive> where TPrimitive : LaneletPrimitiveData
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
        public PrimitiveLayer<LaneletPointData> Points { get; private set; } = new PrimitiveLayer<LaneletPointData>();
        public PrimitiveLayer<LaneletLineStringData> Lines { get; private set; } = new PrimitiveLayer<LaneletLineStringData>();
        public PrimitiveLayer<LaneletData> Lanelets { get; private set; } = new PrimitiveLayer<LaneletData>();
        public PrimitiveLayer<LaneletRegulatoryElement> RegulatoryElements { get; private set; } = new PrimitiveLayer<LaneletRegulatoryElement>();


        /// <summary>
        /// Add lanelet if the same Id does not exist.
        /// </summary>
        /// <param name="lanelet"></param>
        public void Add(LaneletData lanelet)
        {
            if (this.Lanelets.Exists(lanelet.Id))
            {
                return;
            }
            Add(lanelet.LeftBound);
            Add(lanelet.RightBound);
            this.Lanelets.Add(lanelet.Id, lanelet);
        }

        /// <summary>
        /// Add line if the same Id does not exist.
        /// </summary>
        /// <param name="lanelet"></param>
        public void Add(LaneletLineStringData line)
        {
            if (this.Lines.Exists(line.Id))
            {
                return;
            }
            foreach (var point in line.Points)
            {
                Add(point);
            }
            this.Lines.Add(line.Id, line);
        }

        /// <summary>
        /// Add point if the same Id does not exist.
        /// </summary>
        /// <param name="lanelet"></param>
        public void Add(LaneletPointData point)
        {
            if (this.Points.Exists(point.Id))
            {
                return;
            }
            this.Points.Add(point.Id, point);
        }

        /// <summary>
        /// Add regulatory element if the same Id does not exist.
        /// </summary>
        /// <param name="regElem"></param>
        public void Add(LaneletRegulatoryElement regElem)
        {
            if (this.RegulatoryElements.Exists(regElem.Id))
            {
                return;
            }
            this.RegulatoryElements.Add(regElem.Id, regElem);
        }
    }
}
