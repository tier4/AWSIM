using UnityEngine;

namespace AWSIM.Geographic
{
    [System.Serializable]
    public class GeoCoordinate
    {
        [SerializeField]
        private double latitude;
        [SerializeField]
        private double longitude;
        [SerializeField]
        private double altitude;

        public double Latitude => latitude;
        public double Longitude => longitude;
        public double Altitude => altitude;

        public GeoCoordinate()
        {
            latitude = 0;
            longitude = 0;
            altitude = 0;
        }

        public GeoCoordinate(double latitude, double longitude, double altitude)
        {
            this.latitude = latitude;
            this.longitude = longitude;
            this.altitude = altitude;
        }
    }
}
