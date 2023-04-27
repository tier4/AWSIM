using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AWSIM;

namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Intersection class used by RandomTraffic.
    /// Traffic light sequences and information on vehicles in the intersection.
    /// </summary>
    [RequireComponent(typeof(BoxCollider))]
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
            public Group Group => group;
            public TrafficLight[] TrafficLights => trafficLights;

            public TrafficLightGroup(Group group, TrafficLight[] trafficLights)
            {
                this.group = group;
                this.trafficLights = trafficLights;
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

            [SerializeField] Group group;
            [SerializeField] TrafficLight[] trafficLights;
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
            public float IntervalSec => intervalSec;

            /// <summary>
            /// Lighting bulb orders applied to TrafficLightGroup.
            /// </summary>
            public GroupLightingOrder[] GroupLightingOrders => groupLightingOrders;

            public LightingSequence(float intervalSec, GroupLightingOrder[] groupLightingOrders)
            {
                this.intervalSec = intervalSec;
                this.groupLightingOrders = groupLightingOrders;
            }

            [SerializeField] float intervalSec;
            [SerializeField] GroupLightingOrder[] groupLightingOrders;
        }

        /// <summary>
        /// Lighting bulb order applied to TrafficLightGroup.
        /// </summary>
        [Serializable]
        class GroupLightingOrder
        {
            public Group Group => group;
            public TrafficLight.BulbData[] BulbData => bulbData;

            public GroupLightingOrder(Group group, TrafficLight.BulbData[] bulbData)
            {
                this.group = group;
                this.bulbData = bulbData;
            }

            public GroupLightingOrder(Group group, TrafficLight.BulbData bulbData)
            {
                this.group = group;
                this.bulbData = new TrafficLight.BulbData[] { bulbData };
            }

            [SerializeField] Group group;
            [SerializeField] TrafficLight.BulbData[] bulbData;
        }

        [SerializeField] LayerMask colliderMask;
        [SerializeField] TrafficLightGroup[] trafficLightGroups;
        [SerializeField] LightingSequence[] lightingSequences;

        /// <summary>
        /// Is the vehicle exist in the intersection?
        /// </summary>
        public bool VehicleExists => triggerEnterCount > 0;

        Dictionary<Group, TrafficLightGroup> trafficLightGroupPairs;
        int triggerEnterCount = 0;
        BoxCollider boxCollider;

        // Start is called before the first frame update
        void Start()
        {
            trafficLightGroupPairs = trafficLightGroups.ToDictionary(x => x.Group);
            StartCoroutine(StartLightingSequences());
        }

        void Reset()
        {
            boxCollider = GetComponent<BoxCollider>();
            boxCollider.isTrigger = true;
            boxCollider.size = new Vector3(6, 2, 6);
            boxCollider.center = new Vector3(0, 1.1f, 0);

            // default sequence
            var defaultSequences = DefaultLightingSequences();
            lightingSequences = defaultSequences;

            // layer and collider mask
            var vehiclelayer = LayerMask.NameToLayer(Constants.Layers.Vehicle);
            colliderMask.value = 1 << vehiclelayer;

            // default traffic light groups
            trafficLightGroups = new TrafficLightGroup[]
            {
                new TrafficLightGroup(Group.VehicleTrafficLightGroup1, Array.Empty<TrafficLight>()),
                new TrafficLightGroup(Group.VehicleTrafficLightGroup2, Array.Empty<TrafficLight>()),
                new TrafficLightGroup(Group.PedestrianTrafficLightGroup1, Array.Empty<TrafficLight>()),
                new TrafficLightGroup(Group.PedestrianTrafficLightGroup2, Array.Empty<TrafficLight>()),
            };
        }

        private void OnTriggerEnter(Collider other)
        {
            if (CompareLayer(colliderMask, other.gameObject.layer))
            {
                triggerEnterCount++;
            }
        }

        private void OnTriggerExit(Collider other)
        {
            if (CompareLayer(colliderMask, other.gameObject.layer))
            {
                triggerEnterCount--;

                if (triggerEnterCount < 0)  // fail safe
                    triggerEnterCount = 0;
            }
        }

        IEnumerator StartLightingSequences()
        {
            int i = 0;
            while (true)
            {
                foreach (var groupCommand in lightingSequences[i].GroupLightingOrders)
                {
                    var trafficLightGroup = trafficLightGroupPairs[groupCommand.Group];
                    trafficLightGroup.TurnOffAllLights();
                    trafficLightGroup.SetBulbData(groupCommand.BulbData);
                }

                yield return new WaitForSeconds(lightingSequences[i].IntervalSec);

                i++;
                if (i == lightingSequences.Length)
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
                TrafficLight.BulbType.GREEN_BULB,
                TrafficLight.BulbColor.GREEN,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup1, bulbData);
            orderList.Add(order);

            // Vehicle group2 Red.
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RED_BULB,
                TrafficLight.BulbColor.RED,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);

            // Pedestrian group1 Green.
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GREEN_BULB,
                TrafficLight.BulbColor.GREEN,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup1, bulbData);
            orderList.Add(order);

            // Pedestrian group2 Red.
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RED_BULB,
                TrafficLight.BulbColor.RED,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup2, bulbData);
            orderList.Add(order);

            // Create Sequence.
            sequnece = new LightingSequence(15, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 2
            // - Pedestrian group1 Green flashing.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GREEN_BULB,
                TrafficLight.BulbColor.GREEN,
                TrafficLight.BulbStatus.FLASHING);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 3
            // - Pedestrian group1 Red.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RED_BULB,
                TrafficLight.BulbColor.RED,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(1, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 4
            // - Vehicle group1 Yellow.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.YELLOW_BULB,
                TrafficLight.BulbColor.YELLOW,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 5
            // - Vehicle group1 Yellow flashing.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.YELLOW_BULB,
                TrafficLight.BulbColor.YELLOW,
                TrafficLight.BulbStatus.FLASHING);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 6
            // - Vehicle group1 Red.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RED_BULB,
                TrafficLight.BulbColor.RED,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup1, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(3, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 7
            // - Vehicle & Pedestrian group2 Green.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GREEN_BULB,
                TrafficLight.BulbColor.GREEN,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GREEN_BULB,
                TrafficLight.BulbColor.GREEN,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(15, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 8
            // - Pedestrian group2 Green flashing.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.GREEN_BULB,
                TrafficLight.BulbColor.GREEN,
                TrafficLight.BulbStatus.FLASHING);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 9
            // - Pedestrian group2 Red.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RED_BULB,
                TrafficLight.BulbColor.RED,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.PedestrianTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(1, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 10
            // - Vehicle group2 Yellow.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.YELLOW_BULB,
                TrafficLight.BulbColor.YELLOW,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 11
            // - Vehicle group1 Yellow flashing.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.YELLOW_BULB,
                TrafficLight.BulbColor.YELLOW,
                TrafficLight.BulbStatus.FLASHING);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(5, orderList.ToArray());
            sequenceList.Add(sequnece);

            // sequence 12
            // - Vehicle group1 Red.
            orderList.Clear();
            bulbData = new TrafficLight.BulbData(
                TrafficLight.BulbType.RED_BULB,
                TrafficLight.BulbColor.RED,
                TrafficLight.BulbStatus.SOLID_ON);
            order = new GroupLightingOrder(Group.VehicleTrafficLightGroup2, bulbData);
            orderList.Add(order);
            sequnece = new LightingSequence(3, orderList.ToArray());
            sequenceList.Add(sequnece);

            return sequenceList.ToArray();
        }
    }
}
