using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// Setting the vehicle input via device.
    /// Supported devices include the following
    /// - Keyboard
    /// - Logitech g29
    /// </summary>
    public class VehicleInputSettingsUI : MonoBehaviour
    {
        public enum VehicleDeviceInput
        {
            Keyboard = 0,
            Logitech_g29 = 1,
        }

        [Header("Vehicle Inputs")]
        [SerializeField] VehicleOverrideInputManager overrideInputManager;
        [SerializeField] VehicleKeyboardInput keyboardInput;
        [SerializeField] VehicleG29Input g29Input;

        [Header("UI")]
        [SerializeField] Dropdown dropdown;
        [SerializeField] GameObject explainKeyboard;
        [SerializeField] GameObject explainG29;
        [SerializeField] Text G29ConnectedText;

        private void Start()
        {
            // Initialize setup dropdown
            var initialInput = overrideInputManager.ManuallyInput;

            if (initialInput.GetType() == typeof(VehicleKeyboardInput))
            {
                dropdown.value = (int)VehicleDeviceInput.Keyboard;
            }
            else if (initialInput.GetType() == typeof(VehicleG29Input))
            {
                dropdown.value = (int)VehicleDeviceInput.Logitech_g29;
            }

            var deviceInput = IntToVehicleDeviceIpnut(dropdown.value);
            ChangeUI(deviceInput);
        }

        static private VehicleDeviceInput IntToVehicleDeviceIpnut(int value)
        {
            return (VehicleDeviceInput)value;
        }


        public void OnDropdownValueChanged(int dropdownIndex)
        {
            var deviceInput = IntToVehicleDeviceIpnut(dropdownIndex);
            ChangeUI(deviceInput);
        }

        void ChangeUI(VehicleDeviceInput deviceInput)
        {
            if (deviceInput == VehicleDeviceInput.Keyboard)
            {
                // change input
                overrideInputManager.ManuallyInput = keyboardInput;
                keyboardInput.enabled = true;
                g29Input.enabled = false;

                // change UI
                explainKeyboard.SetActive(true);
                explainG29.SetActive(false);
            }

            else if (deviceInput == VehicleDeviceInput.Logitech_g29)
            {
                // change input
                overrideInputManager.ManuallyInput = g29Input;
                g29Input.enabled = true;
                keyboardInput.enabled = false;

                // change UI
                explainKeyboard.SetActive(false);
                explainG29.SetActive(true);

                if (g29Input.IsConnected)
                {
                    G29ConnectedText.text = "connected";
                    G29ConnectedText.color = Color.green;
                }
                else
                {
                    G29ConnectedText.text = "disconnected";
                    G29ConnectedText.color = Color.red;
                }
            }
        }
    }
}