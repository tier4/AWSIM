using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// This class is responsible for detecting vehicle collisions with other vehicles/pedestrians etc. (anything that has a collider). 
    /// The occurrence of a collision is displayed in the GUI and published as ground truths.
    /// What's more, the details of the collision that occurred are saved to the file.
    /// </summary>
    public class CollisionDetector : MonoBehaviour
    {
        [Serializable]
        class GroundTruths
        {
            private float _timer = 0;
            private IPublisher<std_msgs.msg.Bool> _isCollisionEnterPublisher;
            private IPublisher<std_msgs.msg.Bool> _isCollisionConstantlyOccursPublisher;
            [Header("GroundTruths Settings")]
            //GroundTruth publication frequency and topics
            [SerializeField, Tooltip("GroundTruths publication frequency - number of publications per second"), Range(1.0f, 100.0f)] int publishFrequency = 30;
            [SerializeField, Tooltip("'true' will be published when a new collision occurs (not all the time)")] string collisionEnterTopic = "/ground_truth/is_collision_enter";
            [SerializeField, Tooltip("'true' will be published when the collision occurs all the time")] string collisionConstantlyOccursTopic = "/ground_truth/is_collision_occurs";

            public GroundTruths() { }
            public GroundTruths(QoSSettings qosSettings)
            {
                var qos = qosSettings.GetQoSProfile();
                _isCollisionEnterPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(collisionEnterTopic, qos);
                _isCollisionConstantlyOccursPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(collisionConstantlyOccursTopic, qos);
            }

            ~GroundTruths()
            {
                SimulatorROS2Node.RemovePublisher<std_msgs.msg.Bool>(_isCollisionEnterPublisher);
                SimulatorROS2Node.RemovePublisher<std_msgs.msg.Bool>(_isCollisionConstantlyOccursPublisher);
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

            public void PublishEnter(bool isCollisionEnter)
            {
                _isCollisionEnterPublisher.Publish(new std_msgs.msg.Bool() { Data = isCollisionEnter, });

            }

            public void PublishConstantlyOccurs(bool isCollisionConstantlyOccurs)
            {
                if (NeedToPublish())
                    _isCollisionConstantlyOccursPublisher.Publish(new std_msgs.msg.Bool() { Data = isCollisionConstantlyOccurs, });
            }
        }

        private uint numberOfDetections = 0;
        private String basicLog = "";
        private System.DateTime lastEnterTime;
        private System.DateTime lastCollisionStayTime;
        private bool isCollisionConstantlyOccurs = false;

        [Header("GUI Settings")]
        [SerializeField, Tooltip("Should the number of current collisions be displayed?")] private bool showCollisionsDetectedLabel = true;
        [SerializeField, Tooltip("Should details during a collision be displayed (temporarily)?")] private bool showCurrentCollisionDetails = true;
        [SerializeField, Range(0.5f, 30.0f), Tooltip("For how long should the details of the current collision be displayed?")] double currentCollisionDetailsTimeThreshold = 2.0;

        [Header("Log Settings")]
        [SerializeField, Tooltip("Should collision details and their current number be logged to the file?")] private bool saveLogToFile = true;
        [SerializeField, Tooltip("Name of the file (please add *.log)")] private String logOutFileName = "collisions_detected.log";

        [Header("GroundTruths Settings")]
        [SerializeField] private QoSSettings qosSettings;
        [SerializeField] private GroundTruths groundTruths;

        void Start()
        {
            qosSettings = new QoSSettings();
            lastEnterTime = new System.DateTime(1970, 1, 1);
            lastCollisionStayTime = new System.DateTime(1970, 1, 1);
            groundTruths = new GroundTruths(qosSettings);
            logToFile("Start of logs for this run");
        }

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.C))
                showCollisionsDetectedLabel = !showCollisionsDetectedLabel;
        }

        void FixedUpdate()
        {
            groundTruths.PublishConstantlyOccurs(isCollisionConstantlyOccurs);
            if ((System.DateTime.Now - lastCollisionStayTime).TotalSeconds > Time.deltaTime)
                isCollisionConstantlyOccurs = false;
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

        void OnCollisionEnter(Collision collision)
        {
            groundTruths.PublishEnter(true);
            basicLog = "A collision with a \"" + collision.gameObject.name + "\" object has been detected.";
            String extendedLog = basicLog + " Relative linear velocity " + collision.relativeVelocity;
            extendedLog += ". Position of the hit object " + collision.transform.position;
            extendedLog += ". Current number of detections: " + (numberOfDetections++) + ".";
            logToFile(extendedLog);
            lastEnterTime = System.DateTime.Now;
        }

        void OnCollisionStay(Collision collision)
        {
            isCollisionConstantlyOccurs = true;
            lastCollisionStayTime = System.DateTime.Now;
        }

        bool NeedToDisplayDetails()
        {
            return (System.DateTime.Now - lastEnterTime).TotalSeconds < currentCollisionDetailsTimeThreshold;
        }

        void OnGUI()
        {
            if (showCollisionsDetectedLabel)
            {
                var guiStyle = GUI.skin.GetStyle("Label");
                guiStyle.fontSize = Math.Max(10, (int)(Screen.height / 40));
                guiStyle.fontStyle = FontStyle.Bold;
                guiStyle.alignment = TextAnchor.UpperRight;
                guiStyle.normal.textColor = Color.red;
                if (!showCurrentCollisionDetails || !NeedToDisplayDetails())
                {
                    guiStyle.normal.textColor = Color.white;
                    basicLog = "";
                }
                GUI.Label(new Rect(10, 10, Screen.width - 20, 2 * guiStyle.fontSize), "Number of collisions detected: " + numberOfDetections, guiStyle);
                GUI.Label(new Rect(10, 20 + guiStyle.fontSize, Screen.width - 20, 2 * guiStyle.fontSize), basicLog, guiStyle);
            }

        }
    }

}

