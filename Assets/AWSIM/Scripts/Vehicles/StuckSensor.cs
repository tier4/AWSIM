
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    public class StuckSensor : MonoBehaviour
    {
        //Common
        private bool initialized = false;
        private float timer;
        private std_msgs.msg.Bool stuckMsg;
        private IPublisher<std_msgs.msg.Bool> stuckPublisher;
        [SerializeField] string stuckSensorTopic = "/vehicle/status/is_stuck";
        [SerializeField, Range(1.0f, 100.0f)] int publishFrequency = 30;
        [SerializeField] QoSSettings qosSettings;
        //Collision
        private Collision lastCollisionObject;
        private System.DateTime lastCollisionTimestamp;
        [SerializeField] bool considerCollisionStay;
        [SerializeField, Range(0.1f, 6.0f), Tooltip("After this time [s], the disappearance of the collision - means 'is unstuck'")] double collisionTimeThreshold = 3.0;
        //Distance
        private System.DateTime startDistanceSumTimestamp;
        Rigidbody vehicleRigidbody;
        ISubscription<autoware_auto_control_msgs.msg.AckermannControlCommand> ackermanControlCommandSubscriber;
        private double vehicleCurrentSpeed => vehicleRigidbody.velocity.z;
        private double vehicleTargetSpeed = 0;
        private double traveledDistance = 0;
        private double targetDistance = 0;
        private int distanceExceededCounter = 0;
        private int maxDistanceExceededCounterValue = 5;
        [SerializeField] bool considerDistanceTraveledOverTime;
        [SerializeField] string ackermannControlCommandTopic = "/control/command/control_cmd";
        [SerializeField, Range(0.1f, 3.0f), Tooltip("During this time [s], the distances traveled and expected summed up")] double distanceTimePeriod = 2.0;
        [SerializeField, Range(0.1f, 1.0f), Tooltip("It is the difference in distance [m] between the traveled and the expected (summed up in time distanceTimePeriod), which implies 'is stuck'")] double distanceDifferenceThreshold = 0.40;
        [SerializeField, Range(0.1f, 2.0f), Tooltip("Above this speed 'is stuck' analysis based on distance difference is disabled")] double maxSpeedWhileStuck = 1.0;
        //LongImmobility
        private System.DateTime lastMoveTimestamp;
        private System.DateTime lastImmobilityTimestamp;
        [SerializeField] bool considerLongImmobility;
        [SerializeField, Range(0.1f, 3.0f), Tooltip("If during this time [s] (currentSpeed<0.01 and targetSpeed>0.01) then the vehicle 'is stuck', this time must also elapse to go to 'is unstuck'")] double longImmobilityTimeThreshold = 1.0;

        void Start()
        {
            initialized = true;
            stuckMsg = new std_msgs.msg.Bool();
            stuckMsg.Data = false;
            vehicleRigidbody = GetComponent<Rigidbody>();
            var qos = qosSettings.GetQoSProfile();
            ackermanControlCommandSubscriber
                = SimulatorROS2Node.CreateSubscription<autoware_auto_control_msgs.msg.AckermannControlCommand>(
                    ackermannControlCommandTopic, msg => { vehicleTargetSpeed = msg.Longitudinal.Speed; }, qos);
            stuckPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(stuckSensorTopic, qos);
            vehicleTargetSpeed = 0;
            lastCollisionTimestamp = new System.DateTime(1970, 1, 1);
            startDistanceSumTimestamp = System.DateTime.Now;
            lastMoveTimestamp = System.DateTime.Now;
            lastImmobilityTimestamp = new System.DateTime(1970, 1, 1);
        }

        bool NeedToPublish()
        {
            timer += Time.deltaTime;
            var interval = 1.0f / publishFrequency;
            interval -= 0.00001f;
            if (timer < interval)
                return false;
            timer = 0;
            return true;
        }

        bool IsDistanceDifferenceExceeded()
        {
            if (vehicleCurrentSpeed > maxSpeedWhileStuck || Math.Abs(vehicleTargetSpeed) < 0.01)
                distanceExceededCounter = 0;
            traveledDistance += vehicleCurrentSpeed * Time.deltaTime;
            targetDistance += vehicleTargetSpeed * Time.deltaTime;
            var isThresholdExceeded = targetDistance - traveledDistance > distanceDifferenceThreshold;
            var isEndOfDistanceSummation = (System.DateTime.Now - startDistanceSumTimestamp).TotalSeconds > distanceTimePeriod;
            if (isEndOfDistanceSummation)
            {
                traveledDistance = 0.0;
                targetDistance = 0.0;
                startDistanceSumTimestamp = System.DateTime.Now;
                if (isThresholdExceeded)
                    distanceExceededCounter++;
                else
                    distanceExceededCounter--;
                if (distanceExceededCounter < 0) distanceExceededCounter = 0;
                if (distanceExceededCounter > maxDistanceExceededCounterValue) distanceExceededCounter = maxDistanceExceededCounterValue;
            }
            return distanceExceededCounter > maxDistanceExceededCounterValue / 2;
        }

        void FixedUpdate()
        {
            if (initialized == false)
                return;

            if (Math.Abs(vehicleCurrentSpeed) > 0.01 || Math.Abs(vehicleTargetSpeed) < 0.01)
                lastMoveTimestamp = System.DateTime.Now;

            //case when the collision occurs constantly
            var isStuckCollision = false;
            if (considerCollisionStay)
                isStuckCollision = (System.DateTime.Now - lastCollisionTimestamp).TotalSeconds < collisionTimeThreshold;
            
            //case when vehicle should be moving (receives speed) but vehicle isn't moving as it should
            var isDistanceDifferenceExceeded = false;
            if (considerDistanceTraveledOverTime)
                isDistanceDifferenceExceeded = IsDistanceDifferenceExceeded();

            //case when vehicle should be moving but the distanceDifferenceThreshold is not exceeded
            //becouse vehicleTargetSpeed is very small and vehicleCurrentSpeed==0 for a long time
            var isLongImmobilityExceeded = false;
            if (considerLongImmobility)
            {
                var longImmobility = Math.Abs(vehicleTargetSpeed) > 0.01 && (System.DateTime.Now - lastMoveTimestamp).TotalSeconds > longImmobilityTimeThreshold;
                if(longImmobility) lastImmobilityTimestamp = System.DateTime.Now;
                isLongImmobilityExceeded = (System.DateTime.Now - lastImmobilityTimestamp).TotalSeconds < longImmobilityTimeThreshold;
            }

            stuckMsg.Data = isStuckCollision || isDistanceDifferenceExceeded || isLongImmobilityExceeded;
            if (!NeedToPublish())
                stuckPublisher.Publish(stuckMsg);
        }

        void OnGUI()
        {
            if (initialized && stuckMsg.Data)
            {
                var guiStyle = GUI.skin.GetStyle("Label");
                guiStyle.fontSize = Math.Max(10, (int)(Screen.height / 40));
                guiStyle.fontStyle = FontStyle.Bold;
                guiStyle.alignment = TextAnchor.UpperRight;
                guiStyle.normal.textColor = Color.white;
                GUI.Label(new Rect(10, 30 + 2 * guiStyle.fontSize, Screen.width - 20, 2 * guiStyle.fontSize), name + " is stuck", guiStyle);
            }
        }

        void OnCollisionStay(Collision collision)
        {
            lastCollisionTimestamp = System.DateTime.Now;
            lastCollisionObject = collision;
        }
    }

}

