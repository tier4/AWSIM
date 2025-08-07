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
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Awsim.Entity;

namespace Awsim.Usecase.TrafficSimulation
{
    /// <summary>
    /// Intersection class used by RandomTraffic.
    /// Traffic light sequences and information on vehicles in the intersection.
    /// </summary>
    public class TrafficIntersection : MonoBehaviour
    {
        /// <summary>
        /// Enum to group TrafficLight
        /// </summary>
        enum Group
        {
            VehicleTrafficLightGroup1 = 0,
            VehicleTrafficLightGroup2 = 1,
            VehicleTrafficLightGroup3 = 2,
            VehicleTrafficLightGroup4 = 3,

            PedestrianTrafficLightGroup1 = 10,
            PedestrianTrafficLightGroup2 = 11,
            PedestrianTrafficLightGroup3 = 12,
            PedestrianTrafficLightGroup4 = 13,
        }

        /// <summary>
        /// Grouped TrafficLight.
        /// </summary>
        [Serializable]
        class TrafficLightGroup
        {
            public Group Group => _group;
            public TrafficLight[] TrafficLights => _trafficLights;
            [SerializeField] Group _group;
            [SerializeField] TrafficLight[] _trafficLights;

            public TrafficLightGroup(Group group, TrafficLight[] trafficLights)
            {
                this._group = group;
                this._trafficLights = trafficLights;
            }

            public void Initialize()
            {
                foreach (var e in TrafficLights)
                    e.Initialize();
            }

            public void OnUpdate()
            {
                foreach (var e in TrafficLights)
                    e.OnUpdate();
            }

            public void SetBulbData(TrafficLight.BulbData[] bulbData)
            {
                foreach (var trafficLight in TrafficLights)
                {
                    trafficLight.SetBulbData(bulbData);
                }
            }

            public void TurnOffAllLights()
            {
                foreach (var trafficLight in TrafficLights)
                {
                    trafficLight.TurnOffAllBulbs();
                }
            }
        }

        /// <summary>
        /// Lighting sequence of traffic lights bulbs within this intersection.
        /// This sequence is looped.
        /// </summary>
        [Serializable]
        class LightingSequence
        {
            /// <summary>
            /// Time for this sequence (sec)
            /// </summary>
            public float IntervalSec => _intervalSec;

            /// <summary>
            /// Lighting bulb orders applied to TrafficLightGroup.
            /// </summary>
            public GroupLightingOrder[] GroupLightingOrders => _groupLightingOrders;

            public LightingSequence(float intervalSec, GroupLightingOrder[] groupLightingOrders)
            {
                this._intervalSec = intervalSec;
                this._groupLightingOrders = groupLightingOrders;
            }

            [SerializeField] float _intervalSec;

            [SerializeField] GroupLightingOrder[] _groupLightingOrders;
        }

        /// <summary>
        /// Lighting bulb order applied to TrafficLightGroup.
        /// </summary>
        [Serializable]
        class GroupLightingOrder
        {
            public Group Group => _group;
            public TrafficLight.BulbData[] BulbData => _bulbData;

            public GroupLightingOrder(Group group, TrafficLight.BulbData[] bulbData)
            {
                this._group = group;
                this._bulbData = bulbData;
            }

            public GroupLightingOrder(Group group, TrafficLight.BulbData bulbData)
            {
                this._group = group;
                this._bulbData = new TrafficLight.BulbData[] { bulbData };
            }

            [SerializeField] Group _group;

            [SerializeField] TrafficLight.BulbData[] _bulbData;
        }

        [SerializeField] TrafficLightGroup[] _trafficLightGroups;

        [SerializeField] LightingSequence[] _lightingSequences;

        Dictionary<Group, TrafficLightGroup> _trafficLightGroupPairs;

        // Start is called before the first frame update
        public void Initialize()
        {
            _trafficLightGroupPairs = _trafficLightGroups.ToDictionary(x => x.Group);
            foreach (var e in _trafficLightGroups)
                e.Initialize();
            StartCoroutine(StartLightingSequences());
        }

        public void OnUpdate()
        {
            foreach (var e in _trafficLightGroups)
                e.OnUpdate();
        }

        void Reset()
        {
            // default sequence
            var defaultSequences = DefaultLightingSequences();
            _lightingSequences = defaultSequences;

            // default traffic light groups
            _trafficLightGroups = new TrafficLightGroup[]
            {
                new TrafficLightGroup(Group.VehicleTrafficLightGroup1, Array.Empty<TrafficLight>()),
                new TrafficLightGroup(Group.VehicleTrafficLightGroup2, Array.Empty<TrafficLight>()),
                new TrafficLightGroup(Group.PedestrianTrafficLightGroup1, Array.Empty<TrafficLight>()),
                new TrafficLightGroup(Group.PedestrianTrafficLightGroup2, Array.Empty<TrafficLight>()),
            };
        }

        IEnumerator StartLightingSequences()
        {
            int i = 0;
            while (true)
            {
                foreach (var groupCommand in _lightingSequences[i].GroupLightingOrders)
                {
                    var trafficLightGroup = _trafficLightGroupPairs[groupCommand.Group];
                    trafficLightGroup.TurnOffAllLights();
                    trafficLightGroup.SetBulbData(groupCommand.BulbData);
                }

                yield return new WaitForSeconds(_lightingSequences[i].IntervalSec);

                i++;
                if (i == _lightingSequences.Length)
                    i = 0;
            }
        }

        static bool CompareLayer(LayerMask layerMask, int layer)
        {
            return ((1 << layer) & layerMask) != 0;
        }

        static LightingSequence[] DefaultLightingSequences()
        {
            var sequenceList = new List<LightingSequence>();

            TrafficLight.BulbData bulbData;
            GroupLightingOrder order;
            var orderList = new List<GroupLightingOrder>();
            LightingSequence sequnece;

            // sequence 1 :
            // - Vehicle & Pedestrian group1 Green.
            // - Vehicle & Pedestrian group2 Red.

            // Vehicle group1 Green.
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GreenBulb,
                TrafficLight.BulbColor.Green,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup1, bulbData);
            orderList.Add(order);

            // Vehicle group2 Red.
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RedBulb,
                TrafficLight.BulbColor.Red,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);

            // Pedestrian group1 Green.
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GreenBulb,
                TrafficLight.BulbColor.Green,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup1, bulbData);
            orderList.Add(order);

            // Pedestrian group2 Red.
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RedBulb,
                TrafficLight.BulbColor.Red,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup2, bulbData);
            orderList.Add(order);

            // Create Sequence.
            sequnece = new LightingSequence(15, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 2
            // - Pedestrian group1 Green flashing.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GreenBulb,
                TrafficLight.BulbColor.Green,
                TrafficLight.BulbStatus.Frashing);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 3
            // - Pedestrian group1 Red.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RedBulb,
                TrafficLight.BulbColor.Red,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(1, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 4
            // - Vehicle group1 Yellow.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.YellowBulb,
                TrafficLight.BulbColor.Yellow,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 5
            // - Vehicle group1 Yellow flashing.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.YellowBulb,
                TrafficLight.BulbColor.Yellow,
                TrafficLight.BulbStatus.Frashing);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 6
            // - Vehicle group1 Red.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RedBulb,
                TrafficLight.BulbColor.Red,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(3, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 7
            // - Vehicle & Pedestrian group2 Green.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GreenBulb,
                TrafficLight.BulbColor.Green,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GreenBulb,
                TrafficLight.BulbColor.Green,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(15, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 8
            // - Pedestrian group2 Green flashing.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GreenBulb,
                TrafficLight.BulbColor.Green,
                TrafficLight.BulbStatus.Frashing);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 9
            // - Pedestrian group2 Red.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RedBulb,
                TrafficLight.BulbColor.Red,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(1, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 10
            // - Vehicle group2 Yellow.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.YellowBulb,
                TrafficLight.BulbColor.Yellow,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 11
            // - Vehicle group1 Yellow flashing.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.YellowBulb,
                TrafficLight.BulbColor.Yellow,
                TrafficLight.BulbStatus.Frashing);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 12
            // - Vehicle group1 Red.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RedBulb,
                TrafficLight.BulbColor.Red,
                TrafficLight.BulbStatus.SolidOn);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(3, orderList.ToArray());
            sequenceList.Add(sequnece);

            return sequenceList.ToArray();
        }
    }
}
