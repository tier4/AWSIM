using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AWISM;
using ROS2;
using System.Text.RegularExpressions;

namespace AWSIM.Tests
{
    public class MetricsTest : MonoBehaviour
    {
        [SerializeField] Vehicle egoVehicle;
        [SerializeField] float epsilonDistance = 0.2f;
        [SerializeField] float epsilonAngle = 0.1f;
        [SerializeField] GameObject[] testObjects;
        ISubscription<diagnostic_msgs.msg.DiagnosticArray> diagnosticsSubscription;
        diagnostic_msgs.msg.DiagnosticStatus[] expectedCollisionStatuses;
        int collisionIndex = 0;
        Regex vectorRegex = new Regex(@"^\((-?\d+\.\d+), (-?\d+\.\d+), (-?\d+\.\d+)\)$");
        Regex quaternionRegex = new Regex(@"^\((-?\d+\.\d+), (-?\d+\.\d+), (-?\d+\.\d+), (-?\d+\.\d+)\)$");
        bool testSuccess = true;

        IEnumerator driveVehicle()
        {
            float driveTime = 4.0f;
            float currentDriveTime = 0.0f;
            bool running = true;
            egoVehicle.AutomaticShiftInput = Vehicle.Shift.DRIVE;
            egoVehicle.AccelerationInput = 4.0f;
            egoVehicle.SteerAngleInput = -3.0f;
            while (running)
            {
                if (collisionIndex > 0)
                {
                    testObjects[collisionIndex-1].SetActive(false);
                }

                yield return new WaitForFixedUpdate();
                currentDriveTime += Time.fixedDeltaTime;
                if (currentDriveTime > driveTime)
                {
                    egoVehicle.AccelerationInput = 0.0f;
                    egoVehicle.SteerAngleInput = 0.0f;
                    egoVehicle.AutomaticShiftInput = Vehicle.Shift.PARKING;
                    running = false;
                    yield return null;
                }
            }
        }

        void Start()
        {
            if (egoVehicle == null)
            {
                Debug.LogError("Ego vehicle not set. Aborting");
                return;
            }

            prepareDiagnosticMessage();
            prepareExpectedResults();

            StartCoroutine(driveVehicle());

            diagnosticsSubscription
                = SimulatorROS2Node.CreateSubscription<diagnostic_msgs.msg.DiagnosticArray>("metrics", DiagnosticsCallback);
        }

        void prepareDiagnosticMessage()
        {
            expectedCollisionStatuses = new diagnostic_msgs.msg.DiagnosticStatus[2];
            for (int i = 0; i < 2; ++i)
            {
                expectedCollisionStatuses[i] = new diagnostic_msgs.msg.DiagnosticStatus();
                expectedCollisionStatuses[i].Name = "Metrics";
                expectedCollisionStatuses[i].Message = "Ego collision detected";
                expectedCollisionStatuses[i].Hardware_id = "AWSIM";
                expectedCollisionStatuses[i].Level = 1;
                expectedCollisionStatuses[i].Values = new diagnostic_msgs.msg.KeyValue[13];
                for (int j=0; j<13; j++)
                {
                    expectedCollisionStatuses[i].Values[j] = new diagnostic_msgs.msg.KeyValue();
                }
            }
        }

        void prepareExpectedResults()
        {
            expectedCollisionStatuses[0].Values[0].Key = "ego_name";
            expectedCollisionStatuses[0].Values[0].Value = "Lexus RX450h 2015 Sample Sensor";
            expectedCollisionStatuses[0].Values[1].Key = "ego_position";
            expectedCollisionStatuses[0].Values[1].Value = "(-9.0649, -3.1550, -0.0556)";
            expectedCollisionStatuses[0].Values[2].Key = "ego_rotation";
            expectedCollisionStatuses[0].Values[2].Value = "(0.0004, -0.0027, 0.0118, 0.9999)";
            expectedCollisionStatuses[0].Values[3].Key = "ego_velocity";
            expectedCollisionStatuses[0].Values[3].Value = "(3.1176, 0.0644, -0.0424)";
            expectedCollisionStatuses[0].Values[4].Key = "ego_acceleration_input";
            expectedCollisionStatuses[0].Values[4].Value = "4";
            expectedCollisionStatuses[0].Values[5].Key = "ego_steering_input";
            expectedCollisionStatuses[0].Values[5].Value = "-3";
            expectedCollisionStatuses[0].Values[6].Key = "ego_shift_input";
            expectedCollisionStatuses[0].Values[6].Value = "DRIVE";
            expectedCollisionStatuses[0].Values[7].Key = "ego_signal_input";
            expectedCollisionStatuses[0].Values[7].Value = "NONE";
            expectedCollisionStatuses[0].Values[8].Key = "object_name";
            expectedCollisionStatuses[0].Values[8].Value = "Obstacle";
            expectedCollisionStatuses[0].Values[9].Key = "object_position";
            expectedCollisionStatuses[0].Values[9].Value = "(-4.6973, -3.0893, 0.5011)";
            expectedCollisionStatuses[0].Values[10].Key = "object_rotation";
            expectedCollisionStatuses[0].Values[10].Value = "(-0.0007, 0.0085, -0.0026, 1.0000)";
            expectedCollisionStatuses[0].Values[11].Key = "object_velocity";
            expectedCollisionStatuses[0].Values[11].Value = "(1.4892, -0.1502, 0.1102)";
            expectedCollisionStatuses[0].Values[12].Key = "object_type";
            expectedCollisionStatuses[0].Values[12].Value = "ENVIRONMENT";

            expectedCollisionStatuses[1].Values[0].Key = "ego_name";
            expectedCollisionStatuses[1].Values[0].Value = "Lexus RX450h 2015 Sample Sensor";
            expectedCollisionStatuses[1].Values[1].Key = "ego_position";
            expectedCollisionStatuses[1].Values[1].Value = "(3.6367, -1.3192, -0.0722)";
            expectedCollisionStatuses[1].Values[2].Key = "ego_rotation";
            expectedCollisionStatuses[1].Values[2].Value = "(0.0017, -0.011, 0.1277, 0.9917)";
            expectedCollisionStatuses[1].Values[3].Key = "ego_velocity";
            expectedCollisionStatuses[1].Values[3].Value = "(10.5843, 0.2684, -0.2453)";
            expectedCollisionStatuses[1].Values[4].Key = "ego_acceleration_input";
            expectedCollisionStatuses[1].Values[4].Value = "4";
            expectedCollisionStatuses[1].Values[5].Key = "ego_steering_input";
            expectedCollisionStatuses[1].Values[5].Value = "-3";
            expectedCollisionStatuses[1].Values[6].Key = "ego_shift_input";
            expectedCollisionStatuses[1].Values[6].Value = "DRIVE";
            expectedCollisionStatuses[1].Values[7].Key = "ego_signal_input";
            expectedCollisionStatuses[1].Values[7].Value = "NONE";
            expectedCollisionStatuses[1].Values[8].Key = "object_name";
            expectedCollisionStatuses[1].Values[8].Value = "Hatchback";
            expectedCollisionStatuses[1].Values[9].Key = "object_position";
            expectedCollisionStatuses[1].Values[9].Value = "(8.0, 0.0, 0.0)";
            expectedCollisionStatuses[1].Values[10].Key = "object_rotation";
            expectedCollisionStatuses[1].Values[10].Value = "(-0.0027, 0.0158, 0.0001, 0.9999)";
            expectedCollisionStatuses[1].Values[11].Key = "object_velocity";
            expectedCollisionStatuses[1].Values[11].Value = "(2.9999, 0.0000, 0.0002)";
            expectedCollisionStatuses[1].Values[12].Key = "object_type";
            expectedCollisionStatuses[1].Values[12].Value = "NPC_VEHICLE";
        }

        bool TryStringToVector3(string vector3String, out Vector3 vector)
        {
            Match match = vectorRegex.Match(vector3String);
            if (match.Success)
            {
                float x = float.Parse(match.Groups[1].Value);
                float y = float.Parse(match.Groups[2].Value);
                float z = float.Parse(match.Groups[3].Value);
                vector.x = x;
                vector.y = y;
                vector.z = z;
                return true;
            } else {
                vector = Vector3.zero;
                return false;
            }
        }
        bool TryStringToQuaternion(string quaternionString, out Quaternion quaternion)
        {
            Match match = quaternionRegex.Match(quaternionString);
            if (match.Success)
            {
                float x = float.Parse(match.Groups[1].Value);
                float y = float.Parse(match.Groups[2].Value);
                float z = float.Parse(match.Groups[3].Value);
                float w = float.Parse(match.Groups[4].Value);
                quaternion.x = x;
                quaternion.y = y;
                quaternion.z = z;
                quaternion.w = w;
                quaternion = ROS2Utility.RosToUnityRotation(quaternion);
                return true;
            } else {
                quaternion = Quaternion.identity;
                return false;
            }
        }

        bool compareDiagnosticStatus(diagnostic_msgs.msg.DiagnosticStatus a, diagnostic_msgs.msg.DiagnosticStatus b)
        {
            Vector3 testingVectorA = new Vector3();
            Vector3 testingVectorB = new Vector3();
            Quaternion testingQuaternionA = new Quaternion();
            Quaternion testingQuaternionB = new Quaternion();
            if (a.Name != b.Name) return false;
            if (a.Message != b.Message) return false;
            if (a.Hardware_id != b.Hardware_id) return false;
            if (a.Level != b.Level) return false;
            for (int i = 0; i < 13; i++)
            {
                if (a.Values[i].Key != b.Values[i].Key || a.Values[i].Value != b.Values[i].Value)
                {
                    bool isVectorA = TryStringToVector3(a.Values[i].Value, out testingVectorA);
                    bool isVectorB = TryStringToVector3(b.Values[i].Value, out testingVectorB);
                    bool isQuatA = TryStringToQuaternion(a.Values[i].Value, out testingQuaternionA);
                    bool isQuatB = TryStringToQuaternion(b.Values[i].Value, out testingQuaternionB);
                    if (isVectorA && isVectorB)
                    {
                        float distanceAB = (testingVectorB - testingVectorA).magnitude;
                        if (distanceAB > epsilonDistance)
                        {
                            return false;
                        }

                    } else if (isQuatA && isQuatB) {
                        float angle = Quaternion.Angle(testingQuaternionA, testingQuaternionB);
                        if (Mathf.Abs(angle) > epsilonAngle)
                        {
                            return false;
                        }
                    } else {
                        Debug.Log($"<color=red>INVALID STATUS:</color> \"{a.Values[i].Key} != {b.Values[i].Key}\" or \"{a.Values[i].Value} != {b.Values[i].Value}\"");
                        return false;
                    }
                }
            }
            return true;
        }

        void DiagnosticsCallback(diagnostic_msgs.msg.DiagnosticArray msg)
        {
            if (compareDiagnosticStatus(msg.Status[0], expectedCollisionStatuses[collisionIndex]))
            {
                Debug.Log($"<color=green>TEST SUCCESS: </color> Collision {collisionIndex} - metrics are ok!");
            } else {
                testSuccess = false;
                Debug.LogError($"<color=red>TEST FAILED: </color> Collision {collisionIndex} - metrics are not ok!");
            }
            collisionIndex++;

            if (collisionIndex == 2)
            {
                if (testSuccess) {
                    Debug.Log("<color=green>TEST SUCCESS: </color> Test finished!");
                } else {
                    Debug.LogError("<color=red>TEST FAILED: </color> Test finished!");
                }
            }
        }
    }
    
}

