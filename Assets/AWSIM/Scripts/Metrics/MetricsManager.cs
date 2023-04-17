using System.Collections.Generic;
using System.Linq;
using UnityEngine;

using AWSIM;
using AWSIM.Metrics;
using ROS2;

namespace AWISM.Metrics
{
    /// <summary>
    /// Metrics manager.
    /// Manager handles the the metric collision triggers, fills up the metrics and send it over
    /// ROS 2 topic.
    /// </summary>
    public class MetricsManager : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Controllable ego vehicle for gathering the metrics.")]
        Vehicle egoVehicle;

        [SerializeField]
        [Tooltip("Metrics publish topic")]
        string topic = "metrics";

        IPublisher<diagnostic_msgs.msg.DiagnosticArray> metricsPublisher;
        List<MetricsTrigger> metricsTriggers;
        EgoMetrics egoMetrics = new EgoMetrics();
        ObjectMetrics objectMetrics = new ObjectMetrics();
        diagnostic_msgs.msg.DiagnosticArray metricsMessage;

        // Start is called before the first frame update
        void Start()
        {
            metricsTriggers = FindObjectsOfType<MetricsTrigger>().ToList();

            if (egoVehicle == null)
            {
                Debug.LogWarning("Ego vehicle not set. Finding autmatically...");
                egoVehicle = FindObjectOfType<Vehicle>();
                if (egoVehicle == null) 
                {
                    Debug.LogError("No ego vehicle in the scene. Metrics disabled");
                    return;
                }
            }

            var metricsTrigger = egoVehicle.GetComponent<MetricsTrigger>();
            if (metricsTrigger == null)
            {
                metricsTrigger = egoVehicle.gameObject.AddComponent<MetricsTrigger>();
            }
            egoVehicle.gameObject.transform.position.ToString();
            metricsTriggers.Add(metricsTrigger);

            foreach (var trigger in metricsTriggers)
            {
                if (trigger.GetComponent<Vehicle>() == null)
                {
                    Debug.LogWarning("Metrics trigger supports only controllable vehicles.");
                } else {
                    trigger.onCollision += onTriggerCollision;
                }
            }

            metricsPublisher = SimulatorROS2Node.CreatePublisher<diagnostic_msgs.msg.DiagnosticArray>(topic);

            metricsMessage = new diagnostic_msgs.msg.DiagnosticArray();
            metricsMessage.Status = new diagnostic_msgs.msg.DiagnosticStatus[1];
            metricsMessage.Status[0] = new diagnostic_msgs.msg.DiagnosticStatus();

            metricsMessage.Status[0].Name = "Metrics";
            metricsMessage.Status[0].Message = "Ego collision detected";
            metricsMessage.Status[0].Hardware_id = "AWSIM";
            metricsMessage.Status[0].Level = 1;
            metricsMessage.Status[0].Values = new diagnostic_msgs.msg.KeyValue[13];
            for (int i=0; i<13; i++)
            {
                metricsMessage.Status[0].Values[i] = new diagnostic_msgs.msg.KeyValue();
            }
        }

        void onTriggerCollision(Vehicle vehicle, Collision collision)
        {
            egoMetrics.UpdateMetrics(vehicle);
            objectMetrics.UpdateMetrics(collision.gameObject);
            
            updateDiagnosticMessage();
            metricsPublisher.Publish(metricsMessage);
        }

        void updateDiagnosticMessage()
        {
            // Ego
            metricsMessage.Status[0].Values[0].Key = "ego_name";
            metricsMessage.Status[0].Values[0].Value = egoMetrics.name;

            metricsMessage.Status[0].Values[1].Key = "ego_position";
            metricsMessage.Status[0].Values[1].Value = egoMetrics.position.ToString();

            metricsMessage.Status[0].Values[2].Key = "ego_rotation";
            metricsMessage.Status[0].Values[2].Value = egoMetrics.rotation.ToString();

            metricsMessage.Status[0].Values[3].Key = "ego_velocity";
            metricsMessage.Status[0].Values[3].Value = egoMetrics.velocity.ToString();

            metricsMessage.Status[0].Values[4].Key = "ego_acceleration_input";
            metricsMessage.Status[0].Values[4].Value = egoMetrics.accelerationInput.ToString();

            metricsMessage.Status[0].Values[5].Key = "ego_steering_input";
            metricsMessage.Status[0].Values[5].Value = egoMetrics.steeringAngleInput.ToString();

            metricsMessage.Status[0].Values[6].Key = "ego_shift_input";
            metricsMessage.Status[0].Values[6].Value = egoMetrics.shiftGear.ToString();

            metricsMessage.Status[0].Values[7].Key = "ego_signal_input";
            metricsMessage.Status[0].Values[7].Value = egoMetrics.turnSignal.ToString();

            //  Object
            metricsMessage.Status[0].Values[8].Key = "object_name";
            metricsMessage.Status[0].Values[8].Value = objectMetrics.name;

            metricsMessage.Status[0].Values[9].Key = "object_position";
            metricsMessage.Status[0].Values[9].Value = objectMetrics.position.ToString();

            metricsMessage.Status[0].Values[10].Key = "object_rotation";
            metricsMessage.Status[0].Values[10].Value = objectMetrics.rotation.ToString();

            metricsMessage.Status[0].Values[11].Key = "object_velocity";
            metricsMessage.Status[0].Values[11].Value = objectMetrics.velocity.ToString();

            metricsMessage.Status[0].Values[12].Key = "object_type";
            metricsMessage.Status[0].Values[12].Value = objectMetrics.type.ToString();
        }
    } 
}

