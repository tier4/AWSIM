namespace AWSIM.TrafficSimulation
{
    /// <summary>
    /// Config of vehicle behaviors in <see cref="NPCVehicleSimulator"/>.
    /// </summary>
    [System.Serializable]
    public struct NPCVehicleConfig
    {
        /// <summary>
        /// Multiplier used to determine rotational speed from steering angle and vehicle speed.
        /// </summary>
        public const float YawSpeedMultiplier = 0.15f;

        /// <summary>
        /// Rate of change of angular velocity per unit time.<br/>
        /// The higher the value, the faster it can turn, but the more blurred the control becomes.
        /// </summary>
        public const float YawSpeedLerpFactor = 5f;

        /// <summary>
        /// Slow speed at which the vehicle can immediately stop.
        /// </summary>
        public const float SlowSpeed = 5f;

        public float Acceleration;
        public float Deceleration;
        public float SuddenDeceleration;
        public float AbsoluteDeceleration;

        public static NPCVehicleConfig Default()
            => new NPCVehicleConfig
            {
                Acceleration = 3f,
                Deceleration = 2f,
                SuddenDeceleration = 4f,
                AbsoluteDeceleration = 20f
            };
    }
}
