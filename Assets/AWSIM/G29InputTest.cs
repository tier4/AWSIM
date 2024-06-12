using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace AWSIM
{
    public class G29InputTest : MonoBehaviour
    {
        void Update()
        {
            var joystick = Joystick.current;
        }

        public void Throttle(InputAction.CallbackContext context)
        {
            var throttle = context.ReadValue<float>();
            Debug.Log("Throttle : " + throttle);
        }

        public void Brake(InputAction.CallbackContext context)
        {
            var brake = context.ReadValue<float>();
            Debug.Log("Brake : " + brake);
        }

        public void DShift(InputAction.CallbackContext context)
        {
            var dShift = context.ReadValue<float>();
            Debug.Log("DShift : " + dShift);
        }

        public void RShift(InputAction.CallbackContext context)
        {
            var rShift = context.ReadValue<float>();
            Debug.Log("RShift : " + rShift);
        }
    }
}