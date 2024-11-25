using System;
using UnityEngine;

namespace AWSIM
{
    public class FirstOrderLaggedFloat
    {
        private float timeConstant;

        private float desiredValue;
        private float currentValue;
        private float lastTime;

        public float Value
        {
            get
            {
                Update();
                return currentValue;
            }
        }
        public float DesiredValue
        {
            set
            {
                Update();
                desiredValue = value;
            }
            get
            {
                return desiredValue;
            }
        }

        public FirstOrderLaggedFloat(float timeConstant, float initialValue)
        {
            this.timeConstant = timeConstant;

            desiredValue = currentValue = initialValue;
            lastTime = Time.time;
        }

        private void Update()
        {
            float dt = Time.time - lastTime;

            if (timeConstant == 0.0f)
            {
                currentValue = desiredValue;
            }
            else
            {
                currentValue += (dt / timeConstant) * (desiredValue - currentValue);
            }

            lastTime = Time.time;
        }
    }
}