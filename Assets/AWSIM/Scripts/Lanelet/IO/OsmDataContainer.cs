using UnityEngine;

namespace AWSIM.Lanelet
{
    public class OsmDataContainer : ScriptableObject
    {
        [SerializeField]
        private OsmData osmData;

        public OsmData Data { get => osmData; set => osmData = value; }
    }
}
