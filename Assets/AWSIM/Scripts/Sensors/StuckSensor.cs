
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// It allows you to analyze whether there is a permanent collision -> whether the vehicle is stuck
    /// </summary>
    [System.Serializable]
    public class CollisionAnalyzer
    {
        private Collision lastCollisionObject;
        private System.DateTime lastCollisionTimestamp;

        [Header("CollisionStay Settings")]
        //Enable/disable 'is stuck' checking based on CollisionStay
        [SerializeField, Tooltip("Whether the 'is stuck' state should be published when the collision (based on colliders) occurs constantly")] bool considerCollisionStay = true;
        //Threshold for checking if a collision still occurs
        [SerializeField, Range(0.1f, 6.0f), Tooltip("After this time [s], the disappearance of the collision - means 'is unstuck'")] double collisionTimeThreshold = 3.0;

        public CollisionAnalyzer()
        {
            lastCollisionTimestamp = new System.DateTime(1970, 1, 1);
        }

        public void Update(Collision collision)
        {
            lastCollisionTimestamp = System.DateTime.Now;
            lastCollisionObject = collision;
        }
        public bool IsConctantCollision()
        {
            if (!considerCollisionStay)
                return false;
            return (System.DateTime.Now - lastCollisionTimestamp).TotalSeconds < collisionTimeThreshold;
        }
    }

    /// <summary>
    /// It allows you to analyze whether the movement of the vehicle is as it should -> whether the vehicle is stuck
    /// </summary>
    [System.Serializable]
    public class MovementAnalyzer
    {
        //Distance
        private System.DateTime startDistanceSumTimestamp;
        private double vehicleCurrentSpeed = 0;
        private double vehicleTargetSpeed = 0;
        private double traveledDistance = 0;
        private double targetDistance = 0;
        private int distanceExceededCounter = 0;
        private int maxDistanceExceededCounterValue = 5;

        [Header("DistanceTraveled Settings")]
        //Enable/disable 'is stuck' checking based on a comparison of the target and traveled distances
        [SerializeField, Tooltip("Whether the 'is stuck' state should be published when vehicle should be moving (receives speed) but isn't moving as it should.")] bool considerDistanceTraveledOverTime = false;
        //Time over which the distances traveled and expected are summed up
        [SerializeField, Range(0.1f, 3.0f), Tooltip("During this time [s], the distances traveled and expected summed up")] double distanceTimePeriod = 2.0;
        //The difference in distances that implies 'is stuck'
        [SerializeField, Range(0.1f, 1.0f), Tooltip("It is the difference in distance [m] between the traveled and the expected (summed up in time distanceTimePeriod), which implies 'is stuck'")] double distanceDifferenceThreshold = 0.40;
        //Maximum speed below which 'is stuck' based on distances is checked
        [SerializeField, Range(0.1f, 2.0f), Tooltip("Above this speed 'is stuck' analysis based on distance difference is disabled")] double maxSpeedWhileStuck = 1.0;

        //LongImmobility
        private System.DateTime lastMoveTimestamp;
        private System.DateTime lastImmobilityTimestamp;

        [Header("LongImmobility Settings")]
        //Enable/disable 'is stuck' checking based on long immobility
        [SerializeField, Tooltip("Whether the 'is stuck' state should be published when vehicle should be moving but its current speed is zero for a long time. ")] bool considerLongImmobility = false;
        //Time threshold for long immobility (this time must also elapse to go to 'is unstuck')
        [SerializeField, Range(0.1f, 3.0f), Tooltip("If during this time [s] (currentSpeed<0.01 and targetSpeed>0.01) then the vehicle 'is stuck', this time must also elapse to go to 'is unstuck'")] double longImmobilityTimeThreshold = 1.0;

        public MovementAnalyzer()
        {
            startDistanceSumTimestamp = System.DateTime.Now;
            lastMoveTimestamp = System.DateTime.Now;
            lastImmobilityTimestamp = new System.DateTime(1970, 1, 1);
            vehicleTargetSpeed = 0;
        }

        public void Update(double currentSpeed, double targetSpeed)
        {
            vehicleCurrentSpeed = currentSpeed;
            vehicleTargetSpeed = targetSpeed;

            if (Math.Abs(vehicleCurrentSpeed) > 0.01 || Math.Abs(vehicleTargetSpeed) < 0.01)
                lastMoveTimestamp = System.DateTime.Now;

            if (vehicleCurrentSpeed > maxSpeedWhileStuck || Math.Abs(vehicleTargetSpeed) < 0.01)
                distanceExceededCounter = 0;
        }

        public bool IsDistanceDifferenceExceeded()
        {
            if (!considerDistanceTraveledOverTime)
                return false;

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

        public bool isLongImmobilityExceeded()
        {
            if (!considerLongImmobility)
                return false;

            var dt = (System.DateTime.Now - lastMoveTimestamp).TotalSeconds;
            var longImmobility = Math.Abs(vehicleTargetSpeed) > 0.01 && dt > longImmobilityTimeThreshold;
            if (longImmobility) lastImmobilityTimestamp = System.DateTime.Now;
            return (System.DateTime.Now - lastImmobilityTimestamp).TotalSeconds < longImmobilityTimeThreshold;
        }
    }

    /// <summary>
    /// THIS IS A TEMPORARY IMPLEMENTATION OF A VEHICLE STUCK SENSOR
    /// Determined on the basis of 3 sources of information (one, two or all of them can be used at the same time)
    /// whether the vehicle "is stuck" - this state is published on the specific topic and can be also displayed in the GUI.
    /// </summary>
    public class StuckSensor : MonoBehaviour
    {

        private bool initialized = false;
        private float timer = 0;
        private bool isStuck = false;
        private Vector3 previousTransform = Vector3.zero;
        //ROS
        private ISubscription<autoware_auto_control_msgs.msg.AckermannControlCommand> ackermanControlCommandSubscriber;
        private double subscribedTargetSpeed = 0;
        private IPublisher<std_msgs.msg.Bool> stuckPublisher;
        //Enable/disable displaying of 'is stuck' in the GUI
        [SerializeField, Tooltip("Whether 'is stuck' should be displayed in the GUI")] bool showGuiInformation = true;
        [Header("ROS Communication Settings")]
        //The topic for publication 'is stuck' state
        [SerializeField, Tooltip("On this topic, the 'is_stuck' state is published (as a std_msgs::Bool)")] string stuckSensorTopic = "/vehicle/status/is_stuck";
        //The topic to subscribe to the target speed
        [SerializeField, Tooltip("From this topic, speeds are subscribed to calculate the expected distance.")] string ackermannControlCommandTopic = "/control/command/control_cmd";
        //Frequency of publication 'is stuck' state
        [SerializeField, Range(1.0f, 100.0f), Tooltip("'is_stuck' publication frequency - number of publications per second")] int publishFrequency = 30;
        //QoS settings for communication with ROS"
        [SerializeField, Tooltip("QoS settings for communication with ROS")] QoSSettings qosSettings;
        // Analyzers
        [SerializeField] private CollisionAnalyzer collisionAnalyzer;
        [SerializeField] private MovementAnalyzer movementAnalyzer;

        void Start()
        {
            initialized = true;
            collisionAnalyzer = new CollisionAnalyzer();
            movementAnalyzer = new MovementAnalyzer();
            stuckPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(stuckSensorTopic, qosSettings.GetQoSProfile());
            ackermanControlCommandSubscriber
            = SimulatorROS2Node.CreateSubscription<autoware_auto_control_msgs.msg.AckermannControlCommand>(
                ackermannControlCommandTopic, msg => { subscribedTargetSpeed = msg.Longitudinal.Speed; }, qosSettings.GetQoSProfile());
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

        void FixedUpdate()
        {
            if (initialized == false)
                return;

            var calculatedCurrentSpeed = ((transform.position - previousTransform).magnitude) / Time.deltaTime;
            previousTransform = transform.position;
            movementAnalyzer.Update(calculatedCurrentSpeed, subscribedTargetSpeed);

            //case when the collision occurs constantly
            var isConctantCollision = collisionAnalyzer.IsConctantCollision();

            //case when vehicle should be moving (receives speed) but vehicle isn't moving as it should
            var isDistanceDifferenceExceeded = movementAnalyzer.IsDistanceDifferenceExceeded();

            //case when vehicle should be moving but the distanceDifferenceThreshold is not exceeded
            //becouse vehicleTargetSpeed is very small and vehicleCurrentSpeed==0 for a long time
            var isLongImmobilityExceeded = movementAnalyzer.isLongImmobilityExceeded();

            isStuck = isConctantCollision || isDistanceDifferenceExceeded || isLongImmobilityExceeded;
            if (NeedToPublish())
                stuckPublisher.Publish(new std_msgs.msg.Bool() { Data = isStuck, });
        }

        /// <summary>
        /// TODO: It may be moved to the uGUI canvas in the future!
        /// </summary>
        void OnGUI()
        {
            if (showGuiInformation && initialized && isStuck)
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
            collisionAnalyzer.Update(collision);
        }
    }

}

