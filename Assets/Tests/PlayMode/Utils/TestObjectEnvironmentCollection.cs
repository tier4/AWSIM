using System.Linq;
using UnityEngine;

namespace AWSIM.Tests
{
    public class TestObjectEnvironmentCollection : MonoBehaviour
    {
        [SerializeField]
        private TestObjectEnvironment[] container = default;

        public GameObject GetTestObject(string name)
        {
            TestObjectEnvironment item = container.FirstOrDefault( 
                e => string.CompareOrdinal(e.Name, name) == 0);
            
            if(item != null)
            {
                return item.TestObject;
            }

            return null;
        }

        public GameObject GetTestEnvironment(string name)
        {
            TestObjectEnvironment item = container.FirstOrDefault( 
                e => string.CompareOrdinal(e.Name, name) == 0);

            if(item != null)
            {
                return item.Environment;
            }

            return null;
        }

        public void DisableAll()
        {
            if(container == null || container.Length == 0)
            {
                return;
            }

            foreach(TestObjectEnvironment env in container)
            {
                if(env.Environment != null)
                {
                    env.Environment.SetActive(false);
                }

                if(env.TestObject != null)
                {
                    env.TestObject.SetActive(false);
                }
            }
        }

        [System.Serializable]
        public class TestObjectEnvironment
        {
            public string Name = default;
            public GameObject TestObject = default;
            public GameObject Environment = default;
        }
    }
}

