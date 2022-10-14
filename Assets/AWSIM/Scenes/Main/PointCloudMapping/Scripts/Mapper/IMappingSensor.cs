using PclSharp;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    public interface IMappingSensor
    {
        /// <summary>
        /// Returns a human-readable name of the sensor.
        /// </summary>
        public string GetSensorName();

        /// <summary>
        /// Returns (possibly filtered) XYZI pointcloud in ROS world coordinate frame.
        /// </summary>
        /// <param name="worldOriginROS">An offset to add to each point to translate the world origin</param>
        /// <returns>Point cloud from a single capture of the sensor</returns>
        public PointCloudOfXYZI Capture_XYZI_ROS(Vector3 worldOriginROS);
    }
}
