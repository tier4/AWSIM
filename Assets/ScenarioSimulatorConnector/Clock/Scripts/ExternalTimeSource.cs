using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// External ROS TimeSource.
    /// Note: The objet of type 'ExternalTimeSource' shoul be created from asset menu.
    /// </summary>
    [CreateAssetMenu(fileName = "ExternalTimeSource", menuName = "AWSIM/ROS Time Source/External", order = 0)]
    public class ExternalTimeSource : ScriptableObject, ITimeSource
    {
        [TextArea(3, 10)] [SerializeField] string description = "";

        #region [Variables]

        private int seconds = 0;
        private uint nanoseconds = 0;

        #endregion

        #region [Public Methods]

        public void Initialize()
        {
            seconds = 0;
            nanoseconds = 0;
        }

        public void Dispose()
        {

        }

        public void GetTime(out int seconds, out uint nenoseconds)
        {
            seconds = this.seconds;
            nenoseconds = this.nanoseconds;
        }

        public void SetTime(int seconds, uint nanoseconds)
        {
            this.seconds = seconds;
            this.nanoseconds = nanoseconds;
        }
    
        #endregion
    }
}