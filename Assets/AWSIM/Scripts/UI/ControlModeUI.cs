using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    public class ControlModeUI : MonoBehaviour
    {
        [SerializeField] VehicleOverrideInputManager vehicleOverrideInputManager;
        [SerializeField] Text text;

        void Update()
        {
            text.text = vehicleOverrideInputManager.ControlMode.ToString();
        }
    }
}