using ROS2;

namespace AWSIM
{
    /// <summary>
    /// External ROS TimeSource.
    /// </summary>
    public class ExternalTimeSource : ITimeSource
    {

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
