using UnityEngine;
using UnityEngine.Rendering;

#if USE_URP
using UnityEngine.Rendering.Universal;
#else
using UnityEngine.Rendering.HighDefinition;
#endif

namespace AWSIM
{
    /// <summary>
    /// Camera sensor feature manager class, serves as a user interface for configuring parameters related to camera features.
    /// This is a class that sets the volume or camera parameters. 
    /// the CameraSensorFeatureManager is designed for ease of use. It acts as an interface for the user to enable/disable and control the camera effects.
    /// The problem with controlling camera effects without the CameraSensorFeatureManager is that some effects are accessed in the volume object, 
    /// while others are accessed in the camera object.
    /// There are also effects, such as motion blur, that require the camera object to be enabled. 
    /// The intention of this CameraSensorFeatureManager is to handle all this for the user.
    /// </summary>
    public class CameraSensorFeatureManager : MonoBehaviour
    {
        private enum CameraExposureMode
        {
            AUTO = 0,
            MANUAL = 1,
        }

        private enum RenderingType
        {
            HDRP = 0,
            URP = 1
        }

        #region [Components]

        [Header("Components")]
        [SerializeField] private Camera camera = default;
        [SerializeField] private CameraSensor cameraSensor = default;
        [SerializeField] private Volume volume = default;
        [SerializeField] private VolumeProfile profile = null;

        #endregion

        #region [Settings]

        [Header("Prefab Settings")]
        [Tooltip("The rendering pipeline for which the prefab is intended to be used.")]
        [SerializeField] private RenderingType prefabTargetRenderer = RenderingType.HDRP;

        #endregion

        #region [Image Adjustment"]

        [Header("Image Adjustment")]
        [Tooltip("Hue shift of the color")]
        [SerializeField] private bool hue = false;
        [Range(-180f, 180f)]
        [SerializeField] private float hueValue = 0f;
        
        [Tooltip("Saturation of the color")]
        [SerializeField] private bool saturation = false;
        [Range(-100f, 100f)]
        [SerializeField] private float saturationValue = 0f;
        
        [Tooltip("Contrast of the image")]
        [SerializeField] private bool contrast = false;
        [Range(-100f, 100f)]
        [SerializeField] private float contrastValue = 0f;
        
        [Tooltip("Post Exposure of the image")]
        [SerializeField] private bool postExposure = false;
        [Range(-10f, 10f)]
        [SerializeField] private float postExposureValue = 0f;

        [Tooltip("Sharpness of the image")]
        [SerializeField] private bool sharpness = false;
        [Range(0f, 1f)]
        [SerializeField] private float sharpnessValue = 0f;
        
        #endregion

        #region [Exposure Settings]

        [Header("Exposure Settings")]
        [Tooltip("Camera Exposure Mode. Note: In Auto mode, the camera object will be enabled in hierarchy and it will render each frame.")]
        [SerializeField] private CameraExposureMode exposureMode = CameraExposureMode.MANUAL;

        [Tooltip("Camera sensor ISO")]
        [Range(1f, 12800f)]
        [SerializeField] private int ISO = 200;

        [Tooltip("Camera shutter speed. [1/sec]")]
        [Range(0.001f, 10000f)]
        [SerializeField] private float shutterSpeed = 125;

        [Tooltip("Camera Aperture")]
        [Range(0.7f, 32f)]
        [SerializeField] private float aperture = 16;

        #endregion

        #region [Distortion Correction]

        [Header("Lens Distortion Correction")]
        [Tooltip("Apply Lens Distortion Correction")]
        [SerializeField] private bool distortionCorrection = false;

        #endregion

        #region [Additonal Effect]

        [Header("Additonal Effect")]
        [Tooltip("Enables Bloom Effect")]
        [SerializeField] private bool bloomEffect = false;
        [Range(0f, 1f)]
        [SerializeField] private float bloomValue = 0f;

        [Tooltip("Enables Chromatic Aberration Effect")]
        [SerializeField] private bool chromaticAberration = false;
        [Range(0f, 1f)]
        [SerializeField] private float chromaticAberrationValue = 0f;

        [Tooltip("Enables the depth of field effect. This effect depends on the aperture value of the camera.")]
        [SerializeField] private bool depthOfField = false;
        [Range(0.1f, 100f)]
        [SerializeField] private float focusDistance = 0.1f;

        [Tooltip("Enables Motion Blur Effect. Note: When motion blur is active, the camera object will be enabled in hierarchy and it will render each frame.")]
        [SerializeField] private bool motionBlur = false;

        #endregion

        #region [Effects Component]

        private ColorAdjustments colorAdjustmentsComponent = default;

#if !USE_URP // Exposure effect is only supported by HDRP
        private Exposure exposureComponent = default;
#endif

        private Bloom bloomComponent = default;
        private ChromaticAberration chromaticAberrationComponent = default;
        private DepthOfField depthOfFieldComponent = default;
        private MotionBlur motionBlurComponent = default;

        #endregion

        private bool cameraObjectActive = false;

        private void Awake() 
        {
            if(!profile.TryGet<ColorAdjustments>(out colorAdjustmentsComponent))
            {
                Debug.LogWarning("The required Color Adjustment property is not found in camera volume profile.");
            }

#if !USE_URP // Exposure effect is only supported by HDRP
            if(!profile.TryGet<Exposure>(out exposureComponent))
            {
                Debug.LogWarning("The required Exposure property is not found in camera volume profile.");
            }
#endif

            if(!profile.TryGet<Bloom>(out bloomComponent))
            {
                Debug.LogWarning("The required Bloom property is not found in camera volume profile.");
            }

            if(!profile.TryGet<ChromaticAberration>(out chromaticAberrationComponent))
            {
                Debug.LogWarning("The required ChromaticAberration property is not found in camera volume profile.");
            }
            
            if(!profile.TryGet<DepthOfField>(out depthOfFieldComponent))
            {
                Debug.LogWarning("The required DepthOfField property is not found in camera volume profile.");
            }

            if(!profile.TryGet<MotionBlur>(out motionBlurComponent))
            {
                Debug.LogWarning("The required MotionBlur property is not found in camera volume profile.");
            }
        }

        private void Start()
        {
            // check if camera object have to be enable
            if(motionBlur || exposureMode == CameraExposureMode.AUTO)
            {
                cameraObjectActive = true;
            }
            else
            {
                cameraObjectActive = false;
            }

            camera.gameObject.SetActive(cameraObjectActive);

#if !USE_URP
            // exposure mode
            if(exposureComponent != null)
            {
                if(exposureMode == CameraExposureMode.AUTO)
                {
                    exposureComponent.mode = new ExposureModeParameter(ExposureMode.Automatic, true);
                }
                else
                {
                    exposureComponent.mode = new ExposureModeParameter(ExposureMode.UsePhysicalCamera, true);
                }
            }
#endif

#if !USE_URP
            // exposure settings
            ApplyExposureSettings();
#endif

            // image adjustments
            ApplyImageAdjustments();

            // sharpness
            ApplySharpness();

            // lens distortion correction
            ApplyLensDistortionCorrection();

            // bloom effect
            ApplyBloom();

            // chromatic aberration
            ApplyChromaticAberration();

            // depth of field
            ApplyDepthOfField();

            // motion blur
            ApplyMotionBlur();
        }

#if !USE_URP
        private void ApplyExposureSettings()
        {
            camera.GetComponent<HDAdditionalCameraData>().physicalParameters.iso = ISO;
            camera.GetComponent<HDAdditionalCameraData>().physicalParameters.shutterSpeed = 1f/shutterSpeed;
            camera.GetComponent<HDAdditionalCameraData>().physicalParameters.aperture = aperture;
        }
#endif
    
        private void ApplyImageAdjustments()
        {
            if(colorAdjustmentsComponent == null)
            {
                return;
            }

            if(hue || saturation || contrast || postExposure)
            {
                colorAdjustmentsComponent.active = true;
            }
            else
            {
                colorAdjustmentsComponent.active = false;
            }
        
            colorAdjustmentsComponent.hueShift.overrideState = hue;
            colorAdjustmentsComponent.hueShift.value = hueValue;

            colorAdjustmentsComponent.saturation.overrideState = saturation;
            colorAdjustmentsComponent.saturation.value = saturationValue;

            colorAdjustmentsComponent.contrast.overrideState = contrast;
            colorAdjustmentsComponent.contrast.value = contrastValue;

            colorAdjustmentsComponent.postExposure.overrideState = postExposure;
            colorAdjustmentsComponent.postExposure.value = postExposureValue;
        }
    
        private void ApplySharpness()
        {
            if(cameraSensor == null)
            {
                return;
            }

            if(sharpness)
            {
                cameraSensor.sharpeningStrength = sharpnessValue;
            }
            else
            {
                cameraSensor.sharpeningStrength = 0f;
            }
        }

        private void ApplyLensDistortionCorrection()
        {
            if(cameraSensor == null)
            {
                return;
            }

            cameraSensor.EnableLensDistortionCorrection = distortionCorrection;
        }

        private void ApplyBloom()
        {
            if(bloomComponent != null)
            {
                bloomComponent.active = bloomEffect;
                bloomComponent.intensity.overrideState = bloomEffect;
                bloomComponent.intensity.value = bloomValue;
            }
        }

        private void ApplyChromaticAberration()
        {
            if(chromaticAberrationComponent != null)
            {
                chromaticAberrationComponent.active = chromaticAberration;
                chromaticAberrationComponent.intensity.overrideState = chromaticAberration;
                chromaticAberrationComponent.intensity.value = chromaticAberrationValue;
            }
        }

        private void ApplyDepthOfField()
        {
            if(depthOfFieldComponent != null)
            {
                depthOfFieldComponent.active = depthOfField;
                depthOfFieldComponent.focusDistance.overrideState = depthOfField;
                depthOfFieldComponent.focusDistance.value = focusDistance;

#if USE_URP
                depthOfFieldComponent.aperture.overrideState = depthOfField;
                depthOfFieldComponent.aperture.value = aperture;
#endif

            }
        }

        private void ApplyMotionBlur()
        {
            if(motionBlurComponent != null)
            {
                motionBlurComponent.active = motionBlur;
                motionBlurComponent.intensity.overrideState = motionBlur;

#if USE_URP
                motionBlurComponent.intensity.value = (2f / shutterSpeed) / Time.fixedDeltaTime;
#else
                float shutterSpeed = camera.GetComponent<HDAdditionalCameraData>().physicalParameters.shutterSpeed;
                motionBlurComponent.intensity.value = 2f * shutterSpeed / Time.fixedDeltaTime;
#endif
            }
        }
    }

}
