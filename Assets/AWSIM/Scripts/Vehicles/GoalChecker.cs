
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// This class is supposed to integrate with Autoware and publishes 2 ground truths:
    /// is_goal_started - True if a goal has been received, non-zero speeds are received from 
    ///                     Autoware and vehicle has started move in the AWSIM (RECEIVED→STARTED),
    /// is_goal_arrived - True if the vehicle has arrived at position in AWSIM which is the same 
    ///                     as the goal received from Autoware (STARTED→ARRIVED).
    /// Goal can also have states: STARTED → STOPPED (speeds no longer received) or STARTED → CANCELED 
    /// (new goal received - before arriving at the current one).
    /// Each received goal and its state changes are displayed on the GUI and saved to a file.
    /// </summary>
    public class GoalChecker : MonoBehaviour
    {
        [Serializable]
        class Vehicle
        {
            private double _targetSpeed = 0;
            private int _lastSpeedTimestampSec = 0;
            private Rigidbody _rigidbody;
            private double _currentSpeed => _rigidbody.velocity.z;
            public Vector3 Position => _rigidbody.position;
            public Quaternion Rotation => _rigidbody.rotation;
            public bool IsLastSpeedExpired => (SimulatorROS2Node.GetCurrentRosTime().Sec - _lastSpeedTimestampSec) > 2.0;
            ISubscription<autoware_auto_control_msgs.msg.AckermannControlCommand> ackermanControlCommandSubscriber;
            //Input topic to get target speed
            [SerializeField, Tooltip("From this topic, speeds are subscribed to check whether the vehicle should move")] string ackermannControlCommandTopic = "/control/command/control_cmd";

            private Vehicle() { }
            public Vehicle(Rigidbody rigidbody, QoSSettings qosSettings)
            {
                _rigidbody = rigidbody;
                ackermanControlCommandSubscriber
                = SimulatorROS2Node.CreateSubscription<autoware_auto_control_msgs.msg.AckermannControlCommand>(
                    ackermannControlCommandTopic, msg =>
                    {
                        _targetSpeed = msg.Longitudinal.Speed;
                        _lastSpeedTimestampSec = msg.Stamp.Sec;
                    }, qosSettings.GetQoSProfile());
            }

            public bool IsMovingProperly()
            {
                return (!IsLastSpeedExpired && Math.Abs(_currentSpeed) > 0.01 && Math.Abs(_targetSpeed) > 0.01);
            }

            public void Reset()
            {
                _lastSpeedTimestampSec = 0;
            }
        }

        enum GoalState
        {
            RECEIVED,
            STARTED,
            STOPPED,
            CANCELED,
            ARRIVED
        };

        class Goal
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

            public bool IsRecentlyUpdated(int timeThresholdSec)
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

        [Serializable]
        class GoalsManager
        {
            private Dictionary<int, Goal> _goalsDict;
            public Goal CurrentGoal => _goalsDict[_goalsDict.Count - 1];
            public bool IsAnyGoal => _goalsDict.Count > 0;
            public bool IsGoalReceived => IsAnyGoal && CurrentGoal.State == GoalState.RECEIVED;
            public bool IsGoalStopped => IsAnyGoal && CurrentGoal.State == GoalState.STOPPED;
            public bool IsGoalStarted => IsAnyGoal && CurrentGoal.State == GoalState.STARTED;
            public bool IsGoalArrived => IsAnyGoal && CurrentGoal.State == GoalState.ARRIVED;
            public Dictionary<int, Goal> GoalsDist { get { return _goalsDict; } }
            //Enable/disable save logs
            [SerializeField, Tooltip("Should goal state change logs be saved?")] public bool saveLogToFile = true;
            //Filename to save logs
            [SerializeField, Tooltip("Name of the file (please add *.log)")] private String logOutFileName = "goals.log";

            public GoalsManager()
            {
                _goalsDict = new Dictionary<int, Goal>();
            }

            public void AddGoal(geometry_msgs.msg.PoseStamped goalRos)
            {
                if (IsGoalReceived || IsGoalStarted || IsGoalStopped)
                    SetGoalState(GoalState.CANCELED);

                var goal = new Goal(goalRos.Header.Stamp.Sec, goalRos.Pose.Position, goalRos.Pose.Orientation);
                _goalsDict.Add(_goalsDict.Count, goal);
                SetGoalState(GoalState.RECEIVED);
            }

            public void SetGoalState(GoalState state)
            {
                CurrentGoal.State = state;
                // LogToFile(state + ": Goal " + goalsDict.Count + " (x, z, pitch)=" + currentGoal.ToString()); // unity
                LogToFile(state + ": Goal " + _goalsDict.Count + " (x, y, yaw)=" + CurrentGoal.ToStringROS()); // ros
            }

            void LogToFile(String s)
            {
                if (saveLogToFile)
                {
                    var fileWriter = File.AppendText(logOutFileName);
                    fileWriter.WriteLine("[" + DateTime.Now.ToString("yyyy-MM-dd | HH:mm:ss") + "] " + s);
                    fileWriter.Close();
                }
            }
        }


        // Gui
        //Enable/disible displaying goal state change
        [SerializeField, Tooltip("Should goal state change logs be displayed in the gui?")] public bool displayGoalStateChangeInGui = true;
        //Time for displaying goal state change
        [SerializeField, Range(1, 15), Tooltip("For how many seconds should the goal state change be displayed?")] public int goalStateChangeDisplayTime = 3;

        //Common
        private bool initialized = false;
        [Header("ROS Communication Settings")]
        [SerializeField, Tooltip("QoS settings for communication with ROS")] QoSSettings qosSettings;
        [Header("GoalState Log Settings")]
        [SerializeField] private GoalsManager goalsManager;
        [Header("Vehicle Speed Subscription Settings")]
        [SerializeField] private Vehicle vehicle;


        //Autoware
        private autoware_auto_system_msgs.msg.AutowareState autowareState;
        private bool isAutowareReadyForEngagement => autowareState.State is autoware_auto_system_msgs.msg.AutowareState.WAITING_FOR_ENGAGE;
        private bool isAutowareArrived => autowareState.State == autoware_auto_system_msgs.msg.AutowareState.ARRIVED_GOAL || autowareState.State == autoware_auto_system_msgs.msg.AutowareState.FINALIZING;
        private bool isAutowareDriving => autowareState.State is autoware_auto_system_msgs.msg.AutowareState.DRIVING;
        [Header("AutowareState Settings")]
        //Input topic for autoware state
        [SerializeField, Tooltip("From this topic, the autoware state will be subscribed")] string autowareStateSubscriberTopic = "/autoware/state";
        ISubscription<autoware_auto_system_msgs.msg.AutowareState> autowareStateSubscriber;


        //Goal
        ISubscription<geometry_msgs.msg.PoseStamped> currentGoalSubscriber;
        [Header("Goal Settings")]
        //Input topic for current goal
        [SerializeField, Tooltip("From this topic, the current goal will be subscribed")] string currentGoalTopic = "/planning/mission_planning/echo_back_goal_pose";
        //Orientation accuracy
        [SerializeField, Tooltip("It is the accuracy of the orientation with which the vehicle must reach the given goal +/- [deg]"), Range(0.01f, 1.57f)] double rotationTolerance = 15;
        //Position accuracy
        [SerializeField, Tooltip("It is the accuracy of the position with which the vehicle must reach the given goal +/- [m]"), Range(0.01f, 2.0f)] double positionTolerance = 0.2;


        //GroundTruths
        private float _timer = 0;
        private IPublisher<std_msgs.msg.Bool> _goalArrivedPublisher;
        private IPublisher<std_msgs.msg.Bool> _goalStartedPublisher;
        [Header("GroundTruths Settings")]
        //GroundTruth publication frequency and topics
        [SerializeField, Tooltip("GroundTruths publication frequency - number of publications per second"), Range(1.0f, 100.0f)] int publishFrequency = 30;
        [SerializeField, Tooltip("'true' will be published when the vehicle starts moving after receiving the goal")] string goalArrivedTopic = "/ground_truth/is_goal_arrived";
        [SerializeField, Tooltip("'true' will be published when the vehicle arrives at the goal with the assumed accuracy")] string goalStartedTopic = "/ground_truth/is_goal_started";

        void Start()
        {
            autowareState = new autoware_auto_system_msgs.msg.AutowareState();
            autowareState.State = autoware_auto_system_msgs.msg.AutowareState.INITIALIZING;
            goalsManager = new GoalsManager();
            vehicle = new Vehicle(GetComponent<Rigidbody>(), qosSettings);
            var qos = qosSettings.GetQoSProfile();
            _goalArrivedPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(goalArrivedTopic, qos);
            _goalStartedPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(goalStartedTopic, qos);
            autowareStateSubscriber = SimulatorROS2Node.CreateSubscription<autoware_auto_system_msgs.msg.AutowareState>(
                    autowareStateSubscriberTopic, msg => { autowareState = msg; }, qos);
            currentGoalSubscriber
                = SimulatorROS2Node.CreateSubscription<geometry_msgs.msg.PoseStamped>(
                    currentGoalTopic, msg => { goalsManager.AddGoal(msg); vehicle.Reset(); }, qos);
            initialized = true;
        }

        bool IsGoalArrivedWithTolerance()
        {
            var goal = goalsManager.CurrentGoal;
            Func<double, double, double, bool> almostEqualTo = (x, y, eps) => { return Math.Abs(x - y) < eps; };
            var eq_x = almostEqualTo(goal.Position.x, vehicle.Position.x, positionTolerance);
            var eq_z = almostEqualTo(goal.Position.z, vehicle.Position.z, positionTolerance);
            var eq_rot = almostEqualTo(goal.Rotation.eulerAngles.y, vehicle.Rotation.eulerAngles.y, rotationTolerance);
            return eq_x && eq_z && eq_rot;
        }

        void FixedUpdate()
        {
            if (initialized == false)
                return;

            if (isAutowareDriving && (goalsManager.IsGoalReceived || goalsManager.IsGoalStopped) && vehicle.IsMovingProperly())
                goalsManager.SetGoalState(GoalState.STARTED);

            if (!isAutowareDriving && goalsManager.IsGoalStarted && vehicle.IsLastSpeedExpired)
                goalsManager.SetGoalState(GoalState.STOPPED);

            if (isAutowareArrived && goalsManager.IsGoalStarted && IsGoalArrivedWithTolerance())
                goalsManager.SetGoalState(GoalState.ARRIVED);

            if (goalsManager.IsAnyGoal)
                Publish(goalsManager.IsGoalArrived, goalsManager.IsGoalStarted);
        }

        bool NeedToPublish()
        {
            _timer += Time.deltaTime;
            var interval = 1.0f / publishFrequency;
            interval -= 0.00001f;
            if (_timer < interval)
                return false;
            _timer = 0;
            return true;
        }

        void Publish(bool isGoalArrived, bool isGoalStarted)
        {
            if (initialized && NeedToPublish())
            {
                _goalArrivedPublisher.Publish(new std_msgs.msg.Bool() { Data = isGoalArrived, });
                _goalStartedPublisher.Publish(new std_msgs.msg.Bool() { Data = isGoalStarted, });
            }
        }

        void OnGUI()
        {
            if (initialized && displayGoalStateChangeInGui)
            {
                var guiStyle = GUI.skin.GetStyle("Label");
                guiStyle.fontSize = Math.Max(8, (int)(Screen.height / 60));
                guiStyle.fontStyle = FontStyle.Bold;
                guiStyle.alignment = TextAnchor.UpperRight;
                guiStyle.normal.textColor = Color.white;
                foreach (var item in goalsManager.GoalsDist)
                {
                    if (item.Value.IsRecentlyUpdated(goalStateChangeDisplayTime))
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


