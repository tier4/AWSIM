using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// The Vehicle class.
    /// </summary>
    // ----- VEHICLE DYNAMICS CONCEPT -----
    // This vehicle model was created for Autoware simulation,
    // and assuming that Autoware has already created a gas pedal map,
    // this vehicle model uses acceleration as an input value.
    // It has the following features.
    // - Acceleration (m/s^2) control as per command value.
    // - Turning geometry equivalent to 2-wheel model.
    // - 3D mesh (fbx) as road surface for vehicle driving, gradient resistance.
    // - Vehicle attitude change by unity physX engine.
    //   - Yaw, roll, pitch
    //   - Mass-spring damper suspension (WheelCollider's Suspension)
    //
    // ----- VEHICLE INPUT PARAMETER -----
    // The acceleration or deceleration of the vehicle is determined by AutomaticShiftInput and AccelerationInput.
    // And the vehicle will not move in the opposite direction of the Shift(D or R) input.
    // It has the following parameters
    // - Vehicle gear shift input (AT). PARKING, REVERSE, NEUTRAL, DRIVE.
    // - Acceleration input (m/s^2).
    // - Vehicle steering input. Tire angle (degree).
    // - Vehicle turn signal input. NONE, LEFT, RIGHT, HAZARD.
    // For example, here is how the SAMPLE behaves
    // SAMPLE 1,
    //   Shift = D
    //   Speed = Any value
    //   AccelerationInput > 0
    // it will accelerate with input values. (Gradient resistance is received)
    // --
    // SAMPLE 2,
    //   Shift = D
    //   Speed > 0
    //   AccelerationInput < 0
    // it will decelerate (like a brake.).
    // --
    // SAMPLE 3,
    //   Shift = D
    //   Speed <= 0
    //   AccelerationInput < 0
    // it will continuous stop.
    //--
    // 
    // ----- VEHICLE DYNAMICS PARAMETER -----
    // For more advanced vehicle configuration, the following parameters need to be provided.
    //   [Need Vehicle Params]                            [Using With Unity]
    // - vehicle weight (kg)                            : RigidBody mass
    // - Wheel base (m)                                 : Wheel position
    // - Tread width (m)                                : Wheel position
    // - Center of Mass position (x,y,z) (m)            : Rigidbody center of mass position
    // - Moment of inertia (pitch, roll, yaw) (kgm^2)   : RigiBody ineria
    // - Spring rate (N)                                : Wheel suspension
    // - Damper rate (N/s)                              : Wheel suspension
    // - wheel radius (m)                               : Wheel settings

    // TODO: Write detailed documentation about the vehicle.
    public class Vehicle : MonoBehaviour
    {
        public enum Shift
        {
            PARKING = 0,
            REVERSE = 1,
            NEUTRAL = 2,
            DRIVE = 3,
        }

        public enum TurnSignal
        {
            NONE = 0,
            LEFT = 1,
            RIGHT = 2,
            HAZARD = 3,
        }

        /// <summary>
        /// Axle class.
        /// </summary>
        // TODO: Expanded number of wheels and visual support.
        // TODO: Selecting a wheel to apply steering.
        [Serializable]
        public class Axle
        {
            [SerializeField] Wheel leftWheel;
            [SerializeField] Wheel rightWheel;

            /// <summary>
            /// Left wheel of vehicle
            /// </summary>
            public Wheel LeftWheel => leftWheel;

            /// <summary>
            /// Right wheel of vehicle
            /// </summary>
            public Wheel RightWheel => rightWheel;
        }

        [Header("Vehicle Settings")]

        // Center of mass of rigidbody.
        // If null, the default value will be used.
        [SerializeField] Transform centerOfMassTransform;

        // Set inertia?
        [SerializeField] bool useInertia;

        // Moment of inertia applied to the vehicle (x, y, z).
        // reference sample (in 1998): https://www.researchgate.net/publication/228945609_Measured_Vehicle_Inertial_Parameters-NHTSA
        [SerializeField] Vector3 inertia;

        [Header("Physics Settings (experimental)")]

        // Threshold for Rigidbody Sleep.
        [SerializeField] float sleepVelocityThreshold;

        // Coefficient for prevent skidding while stopping.
        // Applies to each wheel.
        // TODO: A more accurate calculation method.
        [Range(0.05f, 1f)] [SerializeField] float SkiddingCancelRate;

        [Space()]
        [Header("Axles Settings")]
        [SerializeField] Axle frontAxle;
        [SerializeField] Axle rearAxle;

        [Header("Input Settings")]
        // Set value to clamp SteerAngleInput (degree).
        // -MaxSteerAngleInput <= SteerAngleInput <= MaxSteerAngleInput.
        [Range(0.01f, 80)]
        [SerializeField] float MaxSteerAngleInput = 35f;

        // Set value to clamp AccelerationInput (m/s^2).
        // -MaxAccelerationInput <= AccelerationInput <= MaxAccelerationInput.
        [Range(0.01f, 50)]
        [SerializeField] float MaxAccelerationInput = 10;

        [Header("Inputs")]

        /// <summary>
        /// Vehicle gear shift input (AT). PARKING, REVERSE, NEUTRAL, DRIVE.
        /// </summary>
        public Shift AutomaticShiftInput;

        /// <summary>
        /// Acceleration input (m/s^2).
        /// In the plane, output the force that will result in this acceleration.
        /// On a slope, it is affected by the slope resistance, so it does not match the input.
        /// </summary>
        // TODO: Compute first order lag
        public float AccelerationInput;

        /// <summary>
        /// Vehicle steering input. Tire angle (degree)
        /// Negative is left, positive is right turn tire angle.
        /// </summary>
        // TODO: Compute first order lag
        public float SteerAngleInput;

        /// <summary>
        /// Vehicle turn signal input. NONE, LEFT, RIGHT, HAZARD.
        /// </summary>
        public TurnSignal SignalInput;

        /// <summary>
        /// Acceleration(m/s^2) in the local coordinate system of the vehicle.
        /// </summary>
        public Vector3 LocalAcceleration { get; private set; }

        /// <summary>
        /// Vehicle speed (m/s)
        /// </summary>
        public float Speed { get; private set; }

        /// <summary>
        /// Vehicle steering angle (degree)
        /// </summary>
        public float SteerAngle => SteerAngleInput;

        /// <summary>
        /// Vehicle turn signal
        /// </summary>
        public TurnSignal Signal => SignalInput;

        /// <summary>
        /// Vehicle velocity (m/s)
        /// </summary>
        public Vector3 Velocity => m_rigidbody.velocity;

        /// <summary>
        /// Vehcile local velocity (m/s)
        /// </summary>
        public Vector3 LocalVelocity => m_transform.InverseTransformDirection(Velocity);

        /// <summary>
        /// Vehicle angular velocity (rad/s)
        /// </summary>
        public Vector3 AngularVelocity { get; private set; }

        // Cache components.
        Wheel[] wheels;
        Rigidbody m_rigidbody;
        Transform m_transform;

        // Cache previous frame values.
        Vector3 lastVelocity;
        Vector3 lastPosition;
        Quaternion lastRotation;

        void Awake()
        {
            m_rigidbody = GetComponent<Rigidbody>();
            m_transform = transform;
            wheels = new Wheel[] { frontAxle.LeftWheel, frontAxle.RightWheel, rearAxle.LeftWheel, rearAxle.RightWheel };

            // Set center of mass position.
            if (centerOfMassTransform != null)
                m_rigidbody.centerOfMass = m_transform.InverseTransformPoint(centerOfMassTransform.position);

            // Set inertia values.
            if (useInertia)
                m_rigidbody.inertiaTensor = inertia;
        }

        void FixedUpdate()
        {
            // Clamp input values.
            AccelerationInput = Mathf.Clamp(AccelerationInput, -MaxAccelerationInput, MaxAccelerationInput);
            SteerAngleInput = Mathf.Clamp(SteerAngleInput, -MaxSteerAngleInput, MaxSteerAngleInput);

            // Compute vehicle infomation.
            ComputeVehicleState();

            // Update Steering, WheelHit, CancelForceRate of the wheel.
            PreUpdateWheels();

            // Sleep ?
            var sleep = CanSleep();
            UpdateVehicleSleep(sleep);

            if (sleep == false)
            {
                // Update wheel force.
                var acceleration = AccelerationInput;
                UpdateWheelsForce(acceleration);
            }

            // cache value for next frame.
            lastVelocity = m_rigidbody.velocity;
            lastPosition = m_transform.position;
            lastRotation = m_transform.rotation;

            // ----- inner methods -----

            void ComputeVehicleState()
            {
                // Speed.
                Vector3 velocity = Velocity;
                Vector3 forward = m_transform.forward;
                Speed = Vector3.Dot(velocity, forward);

                // Local acceleration.
                Vector3 acceleration = (velocity - lastVelocity) / Time.deltaTime;
                LocalAcceleration = m_transform.InverseTransformDirection(acceleration);

                // Angular velocity.
                AngularVelocity = ((transform.rotation.eulerAngles - lastRotation.eulerAngles) / Time.deltaTime) * Mathf.Deg2Rad;
            }

            void PreUpdateWheels()
            {
                // Steer angle is front-only.
                frontAxle.LeftWheel.UpdateWheelSteerAngle(SteerAngle);
                frontAxle.RightWheel.UpdateWheelSteerAngle(SteerAngle);

                foreach (var wheel in wheels)
                {
                    wheel.UpdateWheelHit();
                    wheel.UpdateSkiddingCancelRate(SkiddingCancelRate);
                }
            }

            bool CanSleep()
            {
                // In parking gear, if the wheel is on the ground, put it to sleep.
                if (IsEachWheelGrounded() && AutomaticShiftInput == Shift.PARKING)
                    return true;

                // Is wheel grounded ? Is less than sleepVelocityThreshold ? Is input gear & acceleration can sleep?
                return (IsEachWheelGrounded() && IsCanSleepVelocity() && IsCanSleepInput());

                // ----- inner methods -----

                // Is wheel grounded ?
                bool IsEachWheelGrounded()
                {
                    foreach (var wheel in wheels)
                    {
                        if (wheel.IsGrounded == false)
                            return false;
                    }
                    return true;
                }

                // Is less than sleepVelocityThreshold ?
                bool IsCanSleepVelocity()
                {
                    if (Mathf.Abs(Velocity.z) < sleepVelocityThreshold)
                        return true;
                    else
                        return false;
                }

                // Is input gear & acceleration can sleep ?
                bool IsCanSleepInput()
                {
                    if (AutomaticShiftInput == Shift.REVERSE)
                    {
                        if (AccelerationInput >= 0)
                            return true;
                        else
                            return false;           // Negative input in reverse gear.
                    }
                    else if (AutomaticShiftInput == Shift.DRIVE)
                    {
                        if (AccelerationInput <= 0)
                            return true;
                        else
                            return false;           // Positive input in drive gear.
                    }
                    else
                        return true;
                }
            }

            void UpdateVehicleSleep(bool isSleep)
            {
                // Vehicle sleep.
                if (isSleep)
                {
                    if (m_rigidbody.IsSleeping())
                        return;

                    m_rigidbody.Sleep();
                    m_rigidbody.constraints = RigidbodyConstraints.FreezeAll;
                }
                else
                {
                    m_rigidbody.constraints = RigidbodyConstraints.None;

                    if (m_rigidbody.IsSleeping())
                    {
                        m_rigidbody.WakeUp();
                    }
                }

                // Wheel sleep.
                foreach (var wheel in wheels)
                {
                    if (wheel.IsSleep != isSleep)
                        wheel.UpdateWheelSleep(isSleep);
                }
            }

            void UpdateWheelsForce(float acceleration)
            {
                if (AutomaticShiftInput == Shift.REVERSE)
                {
                    // Consider the maximum acceleration(positive) to avoid moving forward when in reverse gear.
                    if (Speed > 0)
                    {
                        var maxAcceleration = -Speed / Time.deltaTime;
                        if (acceleration > maxAcceleration)
                            acceleration = maxAcceleration;
                    }
                }
                else if (AutomaticShiftInput == Shift.DRIVE)
                {
                    // Consider minimum acceleration(negative) to avoid retraction when in drive gear
                    if (Speed < 0)
                    {
                        var minAcceleration = -Speed / Time.deltaTime;
                        if (acceleration < minAcceleration)
                            acceleration = minAcceleration;
                    }
                }
                else
                    acceleration = 0;

                // Calculates the force applied to all tires.
                // TODO: More precise straight-line control and turning control.
                var perWheelAcceleration = acceleration / wheels.Length;

                // Update the acceleration output by each wheel.
                foreach (var wheel in wheels)
                    wheel.UpdateWheelForce(perWheelAcceleration);
            }
        }
    }
}