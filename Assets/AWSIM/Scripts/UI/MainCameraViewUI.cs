using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// UI View Class for Camera Rotate Around feature from Camera Follow script.
    /// </summary>
    [RequireComponent(typeof(CanvasGroup))]
    public class MainCameraViewUI : MonoBehaviour
    {
        private CanvasGroup canvasGroup = null;
        private FollowCamera followCamera = null;

        private void Awake()
        {
            canvasGroup = this.GetComponent<CanvasGroup>();
            followCamera = FindObjectOfType<FollowCamera>();
            canvasGroup.alpha = 0.0f;
        }

        void OnEnable()
        {
            if(followCamera != null)
            {
                followCamera.onActivateRotateCameraAround += Show;
            }
        }

        void OnDisable()
        {
            if(followCamera != null)
            {
                followCamera.onActivateRotateCameraAround -= Show;
            }
        }

        private void Show(bool active)
        {
            canvasGroup.alpha = active? 1.0f : 0.0f;
        }

    }

}
