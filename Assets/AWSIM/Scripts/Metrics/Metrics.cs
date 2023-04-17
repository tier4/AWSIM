using UnityEngine;

namespace AWSIM.Metrics
{
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
            position = vehicle.transform.position;
            rotation = vehicle.transform.rotation;
            velocity = vehicle.Velocity;
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


        string deduceObjectType(GameObject go)
        {
            if(go.GetComponent<NPCVehicle>())
            {
                return "NPC_VEHICLE";
            } else

            if(go.GetComponent<NPCPedestrian>())
            {
                return "NPC_PEDESTRIAN";
            }

            return "ENVIRONMENT";
        }

        /// <summary>
        /// Updates the metrics based on the game object state.
        /// <param name="gameObject">Game object state to gather metrics from.</param>
        /// </summary>
        public void UpdateMetrics(GameObject gameObject)
        {
            name = gameObject.name;
            position = gameObject.transform.position;
            rotation = gameObject.transform.rotation;
            var collisionObjectRigidbody = gameObject.GetComponent<Rigidbody>();
            if (collisionObjectRigidbody)
            {
                velocity = collisionObjectRigidbody.velocity;
            } else {
                velocity = Vector3.zero;
            }
            type = deduceObjectType(gameObject);
        }
    }
}