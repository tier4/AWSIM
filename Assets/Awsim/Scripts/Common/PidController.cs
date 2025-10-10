using UnityEngine;

namespace Awsim.Common
{
    public class PidController
    {
        public float Kp { get; set; }
        public float Ki { get; set; }
        public float Kd { get; set; }
        float previousError = 0f;
        float integral = 0f;

        public PidController(float kp, float ki, float kd)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            previousError = 0f;
            integral = 0f;
        }

        public float Compute(float setpoint, float actualValue, float deltaTime)
        {
            float error = setpoint - actualValue;
            integral += error * deltaTime;
            float derivative = (error - previousError) / deltaTime;
            previousError = error;
            float direction = error < 0.0 ? -1.0f : 1.0f;
            var result = Kp * error + Ki * integral + Kd * derivative;
            result = Mathf.Clamp(Mathf.Abs(result), 0, 1) * direction;

            return result;
        }
    }
}