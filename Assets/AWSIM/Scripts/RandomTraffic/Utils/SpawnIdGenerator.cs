using System.Threading;

namespace AWSIM.TrafficSimulation
{
    public static class SpawnIdGenerator
    {
        private static uint _nextId = 0;

        public static uint Generate()
        {
            return  _nextId++;
        }
    }
}
