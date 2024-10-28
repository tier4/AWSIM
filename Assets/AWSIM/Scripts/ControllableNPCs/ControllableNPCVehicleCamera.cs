using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Class for handling the camera view of controllable NPC vehicle.
    /// </summary>
    public class ControllableNPCVehicleCamera : MonoBehaviour
    {
        [Header("Components")]
        [SerializeField] private FollowCamera followCamera = default;
        [SerializeField] private Camera camera = default;

        [Header("Settings")]
        [Range(256, 2048)] 
        [SerializeField] private int textureWidth = 1280;
        
        [Range(256, 2048)] 
        [SerializeField] private int textureHeight = 720;

        private RenderTexture renderTexture = default;
        public RenderTexture RenderTexture
        {
            get => renderTexture;
        }
        
        private Vehicle followedVehicle = default;


        private void Awake() 
        {
            CreateRenderTexture();
            camera.targetTexture = renderTexture;    
        }

        #region [Public Methods]

        /// <summary>
        /// Set vehicle to be followed by the camera.
        /// </summary>
        public void SetFollowVehicle(Vehicle vehicle)
        {
            if(vehicle == null)
            {
                return;
            }

            followedVehicle = vehicle;
            followCamera.target = vehicle.transform;
            camera.enabled = true;
        }

        #endregion

        #region [Private Methods]

        private void CreateRenderTexture()
        {
            renderTexture = new RenderTexture(
                textureWidth, textureHeight, 32, RenderTextureFormat.BGRA32);
        }

        #endregion
    }

}

