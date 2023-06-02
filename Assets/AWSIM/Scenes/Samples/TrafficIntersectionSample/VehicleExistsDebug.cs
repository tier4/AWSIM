using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AWSIM.TrafficSimulation;

namespace AWSIM.Samples
{
    public class VehicleExistsDebug : MonoBehaviour
    {
        [SerializeField] TrafficIntersection intersection;
        [SerializeField] bool vehicleExists;

        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }

        private void FixedUpdate()
        {
            vehicleExists = intersection.VehicleExists;
        }
    }
}