using System;
using UnityEngine;

namespace AWSIM.Loader 
{
    [Serializable]
    public class MapConfiguration
    {
        public string mapName;
    }

    [Serializable]
    public class SimulationConfiguration
    {
        public float timeScale;
        public bool useTraffic;
    }

    [Serializable]
    public class EgoConfiguration
    {
        public string egoVehicleName;
        public Vector3 egoPosition;
        public Vector3 egoEulerAngles;
    }

    [Serializable]
    public class AWSIMConfiguration 
    {
        public MapConfiguration mapConfiguration = new MapConfiguration();
        public EgoConfiguration egoConfiguration = new EgoConfiguration();
        public SimulationConfiguration simulationConfiguration = new SimulationConfiguration();
    }
}