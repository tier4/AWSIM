using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Reflection;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Convert the data output from PerceptionResultSensor to ROS2 msg and Publish.
    /// </summary>
    [RequireComponent(typeof(PerceptionResultSensor))]
    public class PerceptionResultRos2Publisher : MonoBehaviour
    {
        /// <summary>
        /// Topic name in DetectedObject msg.
        /// </summary>
        // public string objectTopic = "/awsim/ground_truth/perception/object_recognition/detection/objects";
        private string objectTopic = "/perception/object_recognition/tracking/objects";
        private string lightTopic = "/perception/traffic_light_recognition/traffic_signals";

        /// <summary>
        /// Object sensor frame id.
        /// </summary>
        public string frameId = "base_link";
        
        /// <summary>
        /// max distance that lidar can detect
        /// </summary>
        [Range(0, 200)]
        public float maxDistance = 200f;

        /// <summary>
        /// QoS settings.
        /// </summary>
        [NonSerialized]
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 1,
        };

        private Dictionary<GameObject, NPC> obj2npc = new Dictionary<GameObject, NPC>();
        [NonSerialized]
        public Dictionary<string, NPC> id2npc = new Dictionary<string, NPC>();

        IPublisher<autoware_auto_perception_msgs.msg.TrackedObjects> objectPublisher;
        IPublisher<autoware_auto_perception_msgs.msg.TrafficSignalArray> lightPublisher;
        autoware_auto_perception_msgs.msg.TrackedObjects objectsMsg;
        PerceptionResultSensor objectSensor;
        
        private Dictionary<long, TrafficLight> trafficLights = new Dictionary<long, TrafficLight>();

        void Start()
        {
            // Get ObjectSensor component.
            objectSensor = GetComponent<PerceptionResultSensor>();

            // Set callback.
            objectSensor.OnOutputData += Publish;

            // Create msg.
            objectsMsg = new autoware_auto_perception_msgs.msg.TrackedObjects();

            // Create publisher.
            var qos = qosSettings.GetQoSProfile();
            objectPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_perception_msgs.msg.TrackedObjects>(objectTopic, qos);
            lightPublisher = SimulatorROS2Node.CreatePublisher<autoware_auto_perception_msgs.msg.TrafficSignalArray>(lightTopic, qos);

            // Get ALl TrafficLights
            GetLights();
        }

        void GetLights() {
            var allTrafficLights = GameObject.FindObjectsOfType<TrafficLight>();
            var trafficLightObjects = FindObjectsOfType<TrafficLightLaneletID>();
            for (int i = 0; i < trafficLightObjects.Length; i++) {
                TrafficLightLaneletID laneletId = trafficLightObjects[i];
                GameObject obj = laneletId.gameObject;
                TrafficLight tl = obj.GetComponent<TrafficLight>();
                if (tl != null && laneletId != null && trafficLights.ContainsKey(laneletId.wayID) == false){
                    trafficLights.Add(laneletId.wayID, tl);
                }
            }
        }

        void Publish(PerceptionResultSensor.OutputData outputData)
        {
            if (outputData == null || outputData.objects == null || outputData.origin == null) return;
            var objectsList = new List<autoware_auto_perception_msgs.msg.TrackedObject>();
            foreach (var detectedObject in outputData.objects)
            {
                if(detectedObject ==null || detectedObject.rigidBody == null || detectedObject.dimension == null || detectedObject.bounds == null) continue;

                if (detectedObject.rigidBody.gameObject.GetComponent<NPC>() != null)
                {   
                    if (obj2npc.ContainsKey(detectedObject.rigidBody.gameObject) == false ){
                        obj2npc.Add(detectedObject.rigidBody.gameObject, detectedObject.rigidBody.gameObject.GetComponent<NPC>());
                    }
                    if (id2npc.ContainsKey(BitConverter.ToString(obj2npc[detectedObject.rigidBody.gameObject].rosuuid.Uuid)) == false){
                        id2npc.Add(BitConverter.ToString(obj2npc[detectedObject.rigidBody.gameObject].rosuuid.Uuid), detectedObject.rigidBody.gameObject.GetComponent<NPC>());
                    }
                }
                var rb = detectedObject.rigidBody;
                var dim = detectedObject.dimension;
                var bou = detectedObject.bounds;
                // Check if detectedObject.dimension and detectedObject.bounds are null           
                float distance = Vector3.Distance(outputData.origin.position, rb.transform.position);
                // if (distance > maxDistance) continue;

                //var obj = new autoware_auto_perception_msgs.msg.TrackedObject();
                var obj = new autoware_auto_perception_msgs.msg.TrackedObject();
                obj.Existence_probability = 1.0f;
                // add UUID 
                PropertyInfo property = obj.GetType().GetProperty("Object_id", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static);
                property.SetValue(obj, obj2npc[detectedObject.rigidBody.gameObject].rosuuid);
                //Debug.Log("UUID:" + BitConverter.ToString(obj.Object_id.Uuid));
                //add classification
                var classification = new autoware_auto_perception_msgs.msg.ObjectClassification();
                {
                switch (detectedObject.classification)
                {
                    case ObjectClassification.ObjectType.UNKNOWN:
                        classification.Label = autoware_auto_perception_msgs.msg.ObjectClassification.UNKNOWN;
                        break;
                    case ObjectClassification.ObjectType.CAR:
                        classification.Label = autoware_auto_perception_msgs.msg.ObjectClassification.CAR;
                        break;
                    case ObjectClassification.ObjectType.TRUCK:
                        classification.Label = autoware_auto_perception_msgs.msg.ObjectClassification.TRUCK;
                        break;
                    case ObjectClassification.ObjectType.BUS:
                        classification.Label = autoware_auto_perception_msgs.msg.ObjectClassification.BUS;
                        break;
                    case ObjectClassification.ObjectType.TRAILER:
                        classification.Label = autoware_auto_perception_msgs.msg.ObjectClassification.TRAILER;
                        break;
                    case ObjectClassification.ObjectType.MOTORCYCLE:
                        classification.Label = autoware_auto_perception_msgs.msg.ObjectClassification.MOTORCYCLE;
                        break;
                    case ObjectClassification.ObjectType.BICYCLE:
                        classification.Label = autoware_auto_perception_msgs.msg.ObjectClassification.BICYCLE;
                        break;
                    case ObjectClassification.ObjectType.PEDESTRIAN:
                        classification.Label = autoware_auto_perception_msgs.msg.ObjectClassification.BUS;
                        break;
                    default:
                        Debug.LogWarning("Unknown classification type");
                        break;
                }

                    
                    classification.Probability = 1.0f;
                }
                obj.Classification = new List<autoware_auto_perception_msgs.msg.ObjectClassification>{classification}.ToArray();

                var kinematics = new autoware_auto_perception_msgs.msg.TrackedObjectKinematics();
                // Add pose
                {
                    Vector3 relativePosition = rb.transform.position - outputData.origin.position;
                    Vector3 transformedPosition = Quaternion.Inverse(outputData.origin.rotation) * relativePosition;
                    var p = ROS2Utility.UnityToRosMGRSPosition(transformedPosition);
                    kinematics.Pose_with_covariance.Pose.Position.X = p.X;
                    kinematics.Pose_with_covariance.Pose.Position.Y = p.Y;
                    kinematics.Pose_with_covariance.Pose.Position.Z = p.Z;
                    // add initial rotation of object
                    var r = ROS2Utility.UnityToRosRotation(Quaternion.Inverse(outputData.origin.rotation) *rb.transform.rotation);
                    kinematics.Pose_with_covariance.Pose.Orientation.X = r.x;
                    kinematics.Pose_with_covariance.Pose.Orientation.Y = r.y;
                    kinematics.Pose_with_covariance.Pose.Orientation.Z = r.z;
                    kinematics.Pose_with_covariance.Pose.Orientation.W = r.w;
                }
                // Add twist
                {   
                    var velocity = obj2npc[detectedObject.rigidBody.gameObject].Velocity;
                    kinematics.Twist_with_covariance.Twist.Linear.X = Mathf.Sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
                    kinematics.Twist_with_covariance.Twist.Linear.Y = 0.0;
                    kinematics.Twist_with_covariance.Twist.Linear.Z = 0.0;
                    var a = obj2npc[detectedObject.rigidBody.gameObject].AngularVelocity;
                    var ros_a = ROS2Utility.UnityToRosPosition(a);
                    kinematics.Twist_with_covariance.Twist.Angular.X = 0.0;
                    kinematics.Twist_with_covariance.Twist.Angular.Y = 0.0;
                    kinematics.Twist_with_covariance.Twist.Angular.Z = ros_a.z;
                }
                // Add covariance
                {
                    //kinematics.Has_position_covariance = true;
                    kinematics.Orientation_availability = autoware_auto_perception_msgs.msg.TrackedObjectKinematics.AVAILABLE;
                    //kinematics.Has_twist = true;
                    //kinematics.Has_twist_covariance = true;
                    // Add covariance 6x6
                    const int size = 6;
                    for (int i = 0; i < size; i++)
                    {
                        kinematics.Pose_with_covariance.Covariance[i * size + i] = 1;
                        kinematics.Twist_with_covariance.Covariance[i * size + i] = 1;
                    }
                }
                obj.Kinematics = kinematics;

                // add shape and footprint
                {
                    var shape = new autoware_auto_perception_msgs.msg.Shape();
                    shape.Type = autoware_auto_perception_msgs.msg.Shape.BOUNDING_BOX;
                    shape.Dimensions.X = dim.x;
                    shape.Dimensions.Y = dim.y;
                    shape.Dimensions.Z = dim.z;
                    var footprints = new geometry_msgs.msg.Polygon();
                    // Assuming Point32 has X, Y, Z properties
                    if(bou.Length > 0){
                        var point1 = new geometry_msgs.msg.Point32() { X = bou[0].x, Y = bou[0].y, Z = 0 };
                        var point2 = new geometry_msgs.msg.Point32() { X = bou[1].x, Y = bou[1].y, Z = 0 };
                        var point3 = new geometry_msgs.msg.Point32() { X = bou[2].x, Y = bou[2].y, Z = 0 };
                        var point4 = new geometry_msgs.msg.Point32() { X = bou[3].x, Y = bou[3].y, Z = 0 };
                        footprints.Points = new[] { point1, point2, point3, point4 };
                    }
                    shape.Footprint = footprints;
                    obj.Shape = shape;
                }
                objectsList.Add(obj);
            }
            // Converts data output from ObjectSensor to ROS2 msg
            objectsMsg.Objects = objectsList.ToArray();
            // Update msg header.
            var header = objectsMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);
            objectsMsg.Header.Frame_id = "map";

            // Publish to ROS2.
            objectPublisher.Publish(objectsMsg);
            //PubishLights();
        }

        void PubishLights() {
            var lights = new List<autoware_auto_perception_msgs.msg.TrafficLight>();

            //TrafficSignal: lights.Signals -> (Map_primitive_id, Lights)
            var trafficsignals = new List<autoware_auto_perception_msgs.msg.TrafficSignal>();

            foreach (var x in trafficLights) {
                var light = x.Value;
                var bulbDatas = light.GetBulbData();
                var id = x.Key;
                var tles = new List<autoware_auto_perception_msgs.msg.TrafficLight>();
                var ts = new autoware_auto_perception_msgs.msg.TrafficSignal();
                ts.Map_primitive_id = (int)id;
                foreach (var bulb in bulbDatas) {
                    var tl = new autoware_auto_perception_msgs.msg.TrafficLight();
                    tl.Color = (Byte)(bulb.Color + 1);
                    //Debug.Log("type:  " + tl.Color.GetType().Name);

                    if (bulb.Type <= TrafficLight.BulbType.GREEN_BULB) {
                        tl.Shape = (Byte)5;
                    } else if (bulb.Type == TrafficLight.BulbType.CROSS_BULB) {
                        tl.Shape = 0;
                    } else {
                        tl.Shape = (Byte)(bulb.Type + 2);
                    }
                    tl.Status = (Byte)(bulb.Status + 13);
                    tl.Confidence = 1.0f;
                    tles.Add(tl);
                }
                ts.Lights = tles.ToArray();
                trafficsignals.Add(ts);
            }
            var lightsMsg = new autoware_auto_perception_msgs.msg.TrafficSignalArray();
            lightsMsg.Signals = trafficsignals.ToArray();
            var header = lightsMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref header);
            lightsMsg.Header.Frame_id = "map";
            //lightPublisher.Publish(lightsMsg);
        
        }

        void OnDestroy()
        {
           SimulatorROS2Node.RemovePublisher<autoware_auto_perception_msgs.msg.TrackedObjects>(objectPublisher);
           SimulatorROS2Node.RemovePublisher<autoware_auto_perception_msgs.msg.TrafficSignalArray>(lightPublisher);
        }
    }
}
