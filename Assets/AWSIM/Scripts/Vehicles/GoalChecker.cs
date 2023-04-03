
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    public class GoalChecker : MonoBehaviour
    {
        private enum GoalState
        {
            RECEIVED,
            STARTED,
            STOPPED,
            CANCELED,
            ARRIVED
        }
        private class Goal
        {

            public Goal(int rosTimestamp, geometry_msgs.msg.Point rosPosition, geometry_msgs.msg.Quaternion rosRotation)
            {
                Timestamp = rosTimestamp;
                ROSPosition = rosPosition;
                ROSRotation = rosRotation;
                Position = ROS2Utility.RosMGRSToUnityPosition(rosPosition);
                Rotation = ROS2Utility.RosToUnityRotation(rosRotation);
            }
            public Vector3 Position { get; private set; }
            public Quaternion Rotation { get; private set; }
            public geometry_msgs.msg.Point ROSPosition { get; private set; }
            public geometry_msgs.msg.Quaternion ROSRotation { get; private set; }
            public int Timestamp { get; private set; }
            public int StateTimestamp { get; private set; }
            private GoalState _state;
            public GoalState State
            {
                get { return _state; }
                set { _state = value; StateTimestamp = SimulatorROS2Node.GetCurrentRosTime().Sec; }
            }

            public bool isRecentlyUpdated(int timeThresholdSec)
            {
                return (SimulatorROS2Node.GetCurrentRosTime().Sec - StateTimestamp) < timeThresholdSec;
            }

            public string ToString()
            {
                return "(" + Position.x + ", " + Position.z + ", " + Rotation.eulerAngles.y + ")";
            }

            public string ToStringROS()
            {
                var rosQuaternion = new Quaternion((float)ROSRotation.X, (float)-ROSRotation.Y, (float)ROSRotation.Z, (float)ROSRotation.W);
                return "(" + ROSPosition.X + ", " + ROSPosition.Y + ", " + rosQuaternion.eulerAngles.z * Math.PI / 180 + ")";
            }
        };
        //Common
        private bool initialized = false;
        private float timer = 0;
        private std_msgs.msg.Bool goalArrivedMsg;
        private std_msgs.msg.Bool goalStartedMsg;
        private IPublisher<std_msgs.msg.Bool> goalArrivedPublisher;
        private IPublisher<std_msgs.msg.Bool> goalStartedPublisher;
        [SerializeField] private String logOutFileName = "goals.log";
        [SerializeField] public bool saveLogToFile = true;
        [SerializeField] public bool showGuiInfo = true;
        [SerializeField, Range(1, 15)] public int goalStateChangeDisplayTime = 3;
        [SerializeField] string goalArrivedTopic = "/ground_truth/is_goal_arrived";
        [SerializeField] string goalStartedTopic = "/ground_truth/is_goal_started";
        [SerializeField, Range(1.0f, 100.0f)] int publishFrequency = 30;
        [SerializeField] QoSSettings qosSettings;
        //Vehicle
        private Rigidbody vehicleRigidbody;
        private Vector3 vehiclePosition => vehicleRigidbody.position;
        private Quaternion vehicleRotation => vehicleRigidbody.rotation;
        private double vehicleCurrentSpeed => vehicleRigidbody.velocity.z;
        //TargetSpeed
        private double vehicleTargetSpeed = 0;
        private int lastSpeedTimestampSec = 0;
        private bool isLastSpeedExpired => (SimulatorROS2Node.GetCurrentRosTime().Sec - lastSpeedTimestampSec) > 2.0;
        ISubscription<autoware_auto_control_msgs.msg.AckermannControlCommand> ackermanControlCommandSubscriber;
        [SerializeField] string ackermannControlCommandTopic = "/control/command/control_cmd";
        //Goal
        private Dictionary<int, Goal> goalsDict;
        private bool isAnyGoal => goalsDict.Count > 0;
        private bool isGoalReceived => goalsDict.Count > 0 && currentGoal.State == GoalState.RECEIVED;
        private bool isGoalStopped => goalsDict.Count > 0 && currentGoal.State == GoalState.STOPPED;
        private bool isGoalStarted => goalsDict.Count > 0 && currentGoal.State == GoalState.STARTED;
        private bool isGoalArrived => goalsDict.Count > 0 && currentGoal.State == GoalState.ARRIVED;
        private Goal currentGoal => goalsDict[goalsDict.Count - 1];
        ISubscription<geometry_msgs.msg.PoseStamped> currentGoalSubscriber;
        [SerializeField] string currentGoalTopic = "/planning/mission_planning/echo_back_goal_pose";
        [SerializeField, Range(0.01f, 1.57f), Tooltip("Rotation tolerance +/- [deg]")] double rotationTolerance = 15;
        [SerializeField, Range(0.01f, 2.0f), Tooltip("Position tolerance +/- [m]")] double positionTolerance = 0.2;
        // /api/external/get/engage [tier4_external_api_msgs/msg/EngageStatus]
        // /api/motion/state [autoware_adapi_v1_msgs/msg/MotionState]
        // /autoware/state [autoware_auto_system_msgs/msg/AutowareState]

        void Start()
        {
            initialized = true;
            goalsDict = new Dictionary<int, Goal>();
            goalArrivedMsg = new std_msgs.msg.Bool();
            goalArrivedMsg.Data = false;
            goalStartedMsg = new std_msgs.msg.Bool();
            goalStartedMsg.Data = false;
            vehicleRigidbody = GetComponent<Rigidbody>();
            var qos = qosSettings.GetQoSProfile();
            goalArrivedPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(goalArrivedTopic, qos);
            goalStartedPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(goalStartedTopic, qos);
            currentGoalSubscriber
                = SimulatorROS2Node.CreateSubscription<geometry_msgs.msg.PoseStamped>(
                    currentGoalTopic, msg => { AddGoal(msg); }, qos);
            ackermanControlCommandSubscriber
                = SimulatorROS2Node.CreateSubscription<autoware_auto_control_msgs.msg.AckermannControlCommand>(
                    ackermannControlCommandTopic, msg =>
                    {
                        vehicleTargetSpeed = msg.Longitudinal.Speed;
                        lastSpeedTimestampSec = msg.Stamp.Sec;
                    }, qos);
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

        void AddGoal(geometry_msgs.msg.PoseStamped goalRos)
        {
            if (isGoalReceived || isGoalStarted || isGoalStopped)
                SetCurrentGoalStatus(GoalState.CANCELED);

            var goal = new Goal(goalRos.Header.Stamp.Sec, goalRos.Pose.Position, goalRos.Pose.Orientation);
            goalsDict.Add(goalsDict.Count, goal);
            SetCurrentGoalStatus(GoalState.RECEIVED);
            lastSpeedTimestampSec = 0;
        }

        void logToFile(String s)
        {
            if (saveLogToFile)
            {
                var fileWriter = File.AppendText(logOutFileName);
                fileWriter.WriteLine("[" + DateTime.Now.ToString("yyyy-MM-dd | HH:mm:ss") + "] " + s);
                fileWriter.Close();
            }
        }
        void SetCurrentGoalStatus(GoalState status)
        {
            currentGoal.State = status;
            // logToFile(status + ": Goal " + goalsDict.Count + " (x, z, pitch)=" + currentGoal.ToString()); // unity
            logToFile(status + ": Goal " + goalsDict.Count + " (x, y, yaw)=" + currentGoal.ToStringROS()); // ros
        }

        bool IsGoalarrived(Goal goal)
        {
            Func<double, double, double, bool> almostEqualTo = (x, y, eps) => { return Math.Abs(x - y) < eps; };
            var eq_x = almostEqualTo(goal.Position.x, vehiclePosition.x, positionTolerance);
            var eq_z = almostEqualTo(goal.Position.z, vehiclePosition.z, positionTolerance);
            var eq_rot = almostEqualTo(goal.Rotation.eulerAngles.y, vehicleRotation.eulerAngles.y, rotationTolerance);
            return eq_x && eq_z && eq_rot;
        }

        void FixedUpdate()
        {
            if (initialized == false)
                return;

            if ((isGoalReceived || isGoalStopped) && !isLastSpeedExpired)
                if (Math.Abs(vehicleCurrentSpeed) > 0.01 && Math.Abs(vehicleTargetSpeed) > 0.01)
                    SetCurrentGoalStatus(GoalState.STARTED);

            if (isGoalStarted && isLastSpeedExpired)
                SetCurrentGoalStatus(GoalState.STOPPED);

            if (isGoalStarted && IsGoalarrived(currentGoal))
                SetCurrentGoalStatus(GoalState.ARRIVED);

            if (isAnyGoal && NeedToPublish())
            {
                goalArrivedPublisher.Publish(new std_msgs.msg.Bool() { Data = isGoalArrived, });
                goalStartedPublisher.Publish(new std_msgs.msg.Bool() { Data = isGoalStarted, });
            }
        }

        void OnGUI()
        {
            if (initialized && showGuiInfo)
            {
                var guiStyle = GUI.skin.GetStyle("Label");
                guiStyle.fontSize = Math.Max(8, (int)(Screen.height / 60));
                guiStyle.fontStyle = FontStyle.Bold;
                guiStyle.alignment = TextAnchor.UpperRight;
                guiStyle.normal.textColor = Color.white;
                foreach (var item in goalsDict)
                {
                    if (item.Value.isRecentlyUpdated(goalStateChangeDisplayTime))
                    {
                        var offset = 0;
                        var guiInfo = item.Value.State + ": Goal " + item.Key + " " + item.Value.ToStringROS();
                        if (item.Value.State is GoalState.CANCELED) { guiStyle.normal.textColor = Color.grey; offset = 0; }
                        if (item.Value.State is GoalState.STOPPED) { guiStyle.normal.textColor = Color.grey; offset = 0; }
                        if (item.Value.State is GoalState.RECEIVED) { guiStyle.normal.textColor = Color.white; offset = 10 + guiStyle.fontSize; }
                        if (item.Value.State is GoalState.STARTED) { guiStyle.normal.textColor = Color.yellow; offset = 20 + 2 * guiStyle.fontSize; }
                        if (item.Value.State is GoalState.ARRIVED) { guiStyle.normal.textColor = Color.green; offset = 20 + 2 * guiStyle.fontSize; }
                        GUI.Label(new Rect(10, 210 + offset + 2 * guiStyle.fontSize, Screen.width - 20, 2 * guiStyle.fontSize), guiInfo, guiStyle);
                    }
                }
            }
        }
    }

}

