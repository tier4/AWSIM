using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM {
    
    public class ObjectSubscriber : MonoBehaviour {

        //Get PerceptionResultRos2Publisher
        public PerceptionResultRos2Publisher perceptionResultRos2Publisher;
        QoSSettings qoSSettings = new QoSSettings();
        string subscribedTopic = "/perception/object_recognition/objects";
        ISubscription<autoware_auto_perception_msgs.msg.PredictedObjects> Subscriber;

        void Start() {
            Subscriber = SimulatorROS2Node.CreateSubscription<autoware_auto_perception_msgs.msg.PredictedObjects>(subscribedTopic, myCallback, qoSSettings.GetQoSProfile());
            perceptionResultRos2Publisher = GetComponent<PerceptionResultRos2Publisher>();
        }

        void myCallback(autoware_auto_perception_msgs.msg.PredictedObjects receivedMsg){
            var objects = receivedMsg.Objects;
            //the first index represents the object path

            // Get TimeStep
            int rosSec = receivedMsg.Header.Stamp.Sec;
            uint rosNanosec = receivedMsg.Header.Stamp.Nanosec;

            int currentSec;
            uint currentNanosec;

            SimulatorROS2Node.TimeSource.GetTime(out currentSec, out currentNanosec);

            for (var i = 0; i < objects.Length; i++){

                List<Vector3> path = new List<Vector3>();
                List<Quaternion> rotation = new List<Quaternion>();
                var confidence = -1f;
                var maxindex = 0;
                for (var j = 0; j < objects[i].Kinematics.Predicted_paths.Length; j++){
                    if (objects[i].Kinematics.Predicted_paths[j].Confidence > confidence){
                        confidence = objects[i].Kinematics.Predicted_paths[j].Confidence;
                        maxindex = j;
                    }
                }
                uint deltaTime = objects[i].Kinematics.Predicted_paths[maxindex].Time_step.Nanosec;
                int first_step = (int)((currentNanosec - rosNanosec) / deltaTime);
                int end_step = first_step + 1;
                float delta = (currentNanosec - rosNanosec) % deltaTime;
                
                for (var j = 0; j < objects[i].Kinematics.Predicted_paths[maxindex].Path.Length; j++){
                    var rosPosition = objects[i].Kinematics.Predicted_paths[maxindex].Path[j].Position;
                    var unityPosition = ROS2Utility.RosMGRSToUnityPosition(rosPosition);
                    var rosRotation = objects[i].Kinematics.Predicted_paths[maxindex].Path[j].Orientation;
                    var unityRotation = ROS2Utility.RosToUnityRotation(rosRotation);
                    path.Add(unityPosition);
                    rotation.Add(unityRotation);
                }
                var uuid = BitConverter.ToString(objects[i].Object_id.Uuid);
                if (perceptionResultRos2Publisher.id2npc[uuid].GetType().Name == "NPCVehicle"){
                    var npcvehicle = (NPCVehicle)perceptionResultRos2Publisher.id2npc[uuid];
                    var currentpostion = npcvehicle.currentPosition;
                    var startPosition =  ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[first_step].Position);
                    var endPosition =  ROS2Utility.RosMGRSToUnityPosition(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step].Position);
                    npcvehicle.outerTargetPoint = Vector3.Slerp(startPosition, endPosition, delta);
                    var startRotation = ROS2Utility.RosToUnityRotation(objects[i].Kinematics.Predicted_paths[maxindex].Path[first_step].Orientation);
                    var endRotation = ROS2Utility.RosToUnityRotation(objects[i].Kinematics.Predicted_paths[maxindex].Path[end_step].Orientation);
                    npcvehicle.outerRotation = Quaternion.Lerp(startRotation, endRotation, delta);
                }
     
            }
        }

        void OnDestroy(){
            SimulatorROS2Node.RemoveSubscription<autoware_auto_perception_msgs.msg.PredictedObjects>(Subscriber);
        }
    }
}