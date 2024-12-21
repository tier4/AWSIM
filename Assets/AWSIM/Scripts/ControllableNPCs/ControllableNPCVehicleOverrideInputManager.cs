using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace AWSIM
{
    /// <summary>
    /// Handles controllable NPC vehicle override inputs.
    /// Only Manually Input components are supported.
    /// </summary>
    public class ControllableNPCVehicleOverrideInputManager : MonoBehaviour
    {
        /// <summary>
        /// Types of supported input devices.
        /// </summary>
        public enum VehicleDeviceInput
        {
            KEYBOARD = 0,
            LOGITECH_G29 = 1,
        }

        /// <summary>
        /// Handler for the vehicle input behaviour.
        /// </summary>
        [System.Serializable]
        public class VehicleDeviceInputHandler
        {
            public VehicleDeviceInput Device;
            public VehicleInputBase Behaviour;
        }

        /// <summary>
        /// Class for debug message related to the vehicle device input.
        /// </summary>
        public class VehicleDeviceInputMessageHandler
        {
            public MessageType Type = MessageType.INFO;
            public string Message = "";

            public enum MessageType
            {
                INFO,
                WARNING,
                ERROR
            }
        }

        /// <summary>
        /// List of inputs used by controllable npc vehicle.
        /// </summary>
        [SerializeField] private List<VehicleDeviceInputHandler> inputHandlers = new List<VehicleDeviceInputHandler>();

        /// <summary>
        /// Inputs used during Manually driving. such as keyboard, controller, etc.
        /// </summary>
        public VehicleInputBase ManuallyInput;

        /// <summary>
        /// Reference to controllable npc vehicle.
        /// </summary>
        [SerializeField] private Vehicle vehicle;

        private void Update() 
        {
            if(vehicle == null)
            {
                return;
            }     

            VehicleInputBase.InputArg inputArg = new VehicleInputBase.InputArg(VehicleControlMode.MANUAL, 0f);

            // Update new input for Manually Inputs.
            ManuallyInput?.OnUpdate(inputArg);

            // Apply input to the vehicle.
            vehicle.AccelerationInput = ManuallyInput.AccelerationInput;
            vehicle.SteerAngleInput = ManuallyInput.SteeringInput;
            vehicle.AutomaticShiftInput = ManuallyInput.ShiftInput;
            vehicle.SignalInput = ManuallyInput.TurnSignalInput;       
        }

        /// <summary>
        /// Set the manual input device of the desired type.
        /// </summary>
        /// <param name="device">Type of input device.</param>
        /// <param name="messageHandler">Debug message handler.</param>
        public void SetManuallyInput(VehicleDeviceInput device, out VehicleDeviceInputMessageHandler messageHandler)
        {
            messageHandler = null;

            VehicleDeviceInputHandler inputHandler = inputHandlers.FirstOrDefault(e => e.Device == device);
            if(inputHandler == null)
            {
                Debug.LogWarning("Vehicle Input for " + device + " device could not be found.");
                return;
            }

            // check if the logitech is connected
            if(inputHandler.Device == VehicleDeviceInput.LOGITECH_G29)
            {
                VehicleG29Input vehicleG29Input = (VehicleG29Input)inputHandler.Behaviour;
                if(vehicleG29Input == null)
                {
                    Debug.LogWarning("Vehicle Input for " + inputHandler.Device + " device is expected to be of type " + 
                        typeof(VehicleG29Input) + ", but it is not.");
                    return;
                }

                if(!vehicleG29Input.IsConnected)
                {
                    messageHandler = new VehicleDeviceInputMessageHandler()
                    {
                        Type = VehicleDeviceInputMessageHandler.MessageType.WARNING,
                        Message = "Disconnected",
                    };
                }
            }

            ManuallyInput.enabled = false;
            ManuallyInput = inputHandler.Behaviour;
            ManuallyInput.enabled = true;
        }
    }

}
