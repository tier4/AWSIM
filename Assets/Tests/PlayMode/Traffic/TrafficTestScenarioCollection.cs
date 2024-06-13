using UnityEngine;
using AWSIM.TrafficSimulation;
using System.Linq;

namespace AWSIM.Tests
{
    public class TrafficTestScenarioCollection : MonoBehaviour
    {
        [SerializeField]
        private TestRouteTrafficItem[] routeTraffics = default;

        public TestRouteTrafficConfiguration[] GetTestRouteTrafficConfigs(string testName)
        {
            TestRouteTrafficItem item = routeTraffics.FirstOrDefault(e => string.CompareOrdinal(e.Name, testName) == 0);
            if(item != null)
            {
                return item.Configs;
            }
            return null;
        }
    }

    [System.Serializable]
    public class TestRouteTrafficItem
    {
        public string Name = default;
        public TestRouteTrafficConfiguration[] Configs = default;
    }

    [System.Serializable]
    public class TestRouteTrafficConfiguration
    {
        public RouteTrafficSimulatorConfiguration Config = default;
        public float SpawnDelay = 0f;
    }

}
