using System;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// UI view dedicated to controllable NPC vehicle.
    /// </summary>
    public class ControllableNPCVehicleUI : MonoBehaviour
    {
        public event Action<ControllableNPCVehicleOverrideInputManager.VehicleDeviceInput> onVehicleDeviceInputChanged;

        [Header("Components")]
        [SerializeField] private RawImage cameraRenderTexture = default;
        [SerializeField] private Text velocityText = default;
        [SerializeField] private Text gearText = default;
        [SerializeField] private Text collisionText = default;
        [SerializeField] private Dropdown inputSelectDropdown = default;
        [SerializeField] private ControllableNPCVehicleDebugInfoUI debugInfo = default;

        private Vehicle vehicle;


        #region [Unity Messages]

        private void Awake()
        {
            debugInfo.Hide();
        }

        private void Update()
        {
            if(vehicle == null)
            {
                SetVelocityText(0f);
                SetGearText(Vehicle.Shift.NEUTRAL);
                return;
            }

            SetVelocityText(vehicle.Speed * 3.6f);
            SetGearText(vehicle.AutomaticShift);
        }

        #endregion

        #region [Public Methods]

        public void SetRenderTexture(RenderTexture renderTexture)
        {
            cameraRenderTexture.texture = renderTexture;
        }

        public void SetVehicle(Vehicle vehicle)
        {
            this.vehicle = vehicle;
        }

        public void SetVehicleInputType(ControllableNPCVehicleOverrideInputManager.VehicleDeviceInput inputType)
        {
            inputSelectDropdown.value = (int)inputType;
        }

        public void SetDebugInfo(bool show, string info, ControllableNPCVehicleOverrideInputManager.VehicleDeviceInputMessageHandler.MessageType messageType)
        {
            if(!show)
            {
                debugInfo.Hide();
            }
            else
            {
                debugInfo.SetDebugInfo(info, messageType);
                debugInfo.Show();
            }
        }

        #endregion

        #region [Public Button Callback]

        public void OnInputSelectDropdownValueChanged(int selected)
        {
            onVehicleDeviceInputChanged?.Invoke((ControllableNPCVehicleOverrideInputManager.VehicleDeviceInput)selected);
        }

        #endregion

        #region [Private Methods]

        private void SetVelocityText(float velocity)
        {
            velocityText.text = Mathf.Floor(velocity).ToString("F0");
        }

        private void SetGearText(Vehicle.Shift shift)
        {
            if(shift == Vehicle.Shift.PARKING)
            {
                gearText.text = "P";
            }
            else if(shift == Vehicle.Shift.DRIVE)
            {
                gearText.text = "D";
            }
            else if(shift == Vehicle.Shift.REVERSE)
            {
                gearText.text = "R";
            }
            else
            {
                gearText.text = "N";
            }
        }
    
        #endregion
    }
}