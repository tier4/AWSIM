using System.Linq;
using UnityEngine;

namespace AWSIM.Tests
{
    public class TrafficTestEnvironmentCollection : MonoBehaviour
    {
        [SerializeField]
        private TrafficTestEnvironment[] container = default;

        public GameObject GetTrafficEnvironment(string name)
        {
            TrafficTestEnvironment item = container.FirstOrDefault( 
                e => string.CompareOrdinal(e.Name, name) == 0);

            if(item != null)
            {
                return item.Environment;
            }

            return null;
        }

        public Vector3 GetCameraPosition(string name)
        {
            TrafficTestEnvironment item = container.FirstOrDefault( 
                e => string.CompareOrdinal(e.Name, name) == 0);

            if(item != null)
            {
                return item.CameraPosition;
            }

            return Vector3.zero;
        }

        public Vector3 GetCameraRotation(string name)
        {
            TrafficTestEnvironment item = container.FirstOrDefault( 
                e => string.CompareOrdinal(e.Name, name) == 0);

            if(item != null)
            {
                return item.CameraRotation;
            }

            return Vector3.zero;
        }

        public void SelectEnvironment(string name)
        {
            if(container == null || container.Length == 0)
            {
                return;
            }

            foreach(TrafficTestEnvironment env in container)
            {
                if(string.CompareOrdinal(env.Name, name) == 0)
                {
                    env.Environment.SetActive(true);
                }
                else
                {
                    env.Environment.SetActive(false);
                }
            }
        }

        [System.Serializable]
        public class TrafficTestEnvironment
        {
            public string Name = default;
            public GameObject Environment = default;

            public Vector3 CameraPosition = default;
            public Vector3 CameraRotation = default;
        }
    }
}
