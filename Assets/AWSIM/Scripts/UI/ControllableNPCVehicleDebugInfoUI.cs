using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// UI component for displaying the debug information on the controllable NPC vehicle view.
    /// </summary>
    public class ControllableNPCVehicleDebugInfoUI : MonoBehaviour
    {
        [Header("Components")]
        [SerializeField] private CanvasGroup canvasGroup = default;
        [SerializeField] private Text debugText = default;

        [Header("Settings")]
        [SerializeField] private Color infoColor = default;
        [SerializeField] private Color warningColor = default;
        [SerializeField] private Color errorColor = default;

        public void Show()
        {
            canvasGroup.alpha = 1.0f;
        }

        public void Hide()
        {
            canvasGroup.alpha = 0.0f;
        }

        public void SetDebugInfo(string text, ControllableNPCVehicleOverrideInputManager.VehicleDeviceInputMessageHandler.MessageType type)
        {
            if(type == ControllableNPCVehicleOverrideInputManager.VehicleDeviceInputMessageHandler.MessageType.ERROR)
            {
                debugText.color = errorColor;
            }
            else if(type == ControllableNPCVehicleOverrideInputManager.VehicleDeviceInputMessageHandler.MessageType.WARNING)
            {
                debugText.color = warningColor;
            }
            else
            {
                debugText.color = infoColor;
            }
            
            debugText.text = text;
        }
    }
}