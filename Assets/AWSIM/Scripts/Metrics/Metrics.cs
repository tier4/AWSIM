using UnityEngine;

namespace AWSIM.Metrics
{
    enum ObjectType {
        NPC_PEDESTRIAN = 0,
        NPC_VEHICLE = 1,
        ENVIRONMENT = 2,
        UNKNOWN = 3
    }
    public class EgoMetrics {
        public string name;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 velocity;
        public float accelerationInput;
        public float steeringAngleInput;
        public Vehicle.Shift shiftGear;
        public Vehicle.TurnSignal turnSignal;

        /// <summary>
        /// Updates the metrics based on the Vehicle state.
        /// <param name="vehicle">Vehicle state to gather metrics from.</param>
        /// </summary>
        public void UpdateMetrics(Vehicle vehicle)
        {
            name = vehicle.name;
            position = ROS2Utility.UnityToRosPosition(vehicle.transform.position);
            rotation = ROS2Utility.UnityToRosRotation(vehicle.transform.rotation);
            velocity = ROS2Utility.UnityToRosPosition(vehicle.transform.InverseTransformDirection(vehicle.lastVelocity));
            accelerationInput = vehicle.AccelerationInput;
            steeringAngleInput = vehicle.SteerAngleInput;
            shiftGear = vehicle.AutomaticShiftInput;
            turnSignal = vehicle.SignalInput;
        }
    }

    public class ObjectMetrics {
        public string name;
        public string type;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 velocity;
        ObjectType deduceObjectType(GameObject go)
        {
            if(go.GetComponent<NPCVehicle>())
            {
                return ObjectType.NPC_VEHICLE;
            } else
            if(go.GetComponent<NPCPedestrian>())
            {
                return ObjectType.NPC_PEDESTRIAN;
            }

            return ObjectType.ENVIRONMENT;
        }

        /// <summary>
        /// Updates the metrics based on the game object state.
        /// <param name="gameObject">Game object state to gather metrics from.</param>
        /// </summary>
        public void UpdateMetrics(GameObject gameObject)
        {
            name = gameObject.name;
            position = ROS2Utility.UnityToRosPosition(gameObject.transform.position);
            rotation = ROS2Utility.UnityToRosRotation(gameObject.transform.rotation);
            var objectType = deduceObjectType(gameObject); 
            type = objectType.ToString();

            switch (objectType)
            {
                case (ObjectType.NPC_VEHICLE):
                    velocity = ROS2Utility.UnityToRosPosition(gameObject.GetComponent<NPCVehicle>().lastVelocity);
                    break;
                default:
                    var collisionObjectRigidbody = gameObject.GetComponent<Rigidbody>();
                    if (collisionObjectRigidbody)
                    {
                        velocity = ROS2Utility.UnityToRosPosition(collisionObjectRigidbody.velocity);
                    } else {
                        velocity = Vector3.zero;
                    }
                    break;
            }
        }
    }
}