using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// Setting UI for FollowCamera
    /// </summary>
    public class FollowCameraSettingsUI : MonoBehaviour
    {
        [SerializeField] FollowCamera followCamera;
        [SerializeField] Toggle enableRotateAroundModeToggle;

        void Start()
        {
            enableRotateAroundModeToggle.isOn = followCamera.EnableRotateAroundMode;
        }

        public void OnToggle(bool isOn)
        {
            followCamera.EnableRotateAroundMode = isOn;
        }
    }
}