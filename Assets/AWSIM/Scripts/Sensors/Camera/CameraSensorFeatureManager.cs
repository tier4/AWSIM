using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace AWSIM
{
    /// <summary>
    /// Camera sensor feature manager class, serves as a user interface for configuring parameters related to camera features.
    /// </summary>
    public class CameraSensorFeatureManager : MonoBehaviour
    {
        private enum CameraExposureMode
        {
            AUTO = 0,
            MANUAL = 1,
        }

        #region [Components]

        [Header("Components")]
        [SerializeField] private Camera camera = default;
        [SerializeField] private CameraSensor cameraSensor = default;
        [SerializeField] private Volume volume = default;
        [SerializeField] private VolumeProfile profile = null;

        #endregion

        #region [Image Adjustment"]

        [Header("Image Adjustment")]
        [SerializeField] private bool hue = false;
        [Range(-180f, 180f)]
        [SerializeField] private float hueValue = 0f;
        
        [SerializeField] private bool saturation = false;
        [Range(-100f, 100f)]
        [SerializeField] private float saturationValue = 0f;
        
        [SerializeField] private bool contrast = false;
        [Range(-100f, 100f)]
        [SerializeField] private float contrastValue = 0f;
        
        [SerializeField] private bool postExposure = false;
        [Range(-10f, 10f)]
        [SerializeField] private float postExposureValue = 0f;

        [SerializeField] private bool sharpness = false;
        [Range(0f, 1f)]
        [SerializeField] private float sharpnessValue = 0f;
        
        #endregion

        #region [Exposure Settings]

        [Header("Exposure Settings")]
        [SerializeField] private CameraExposureMode exposureMode = CameraExposureMode.MANUAL;

        [Range(1f, 12800f)]
        [SerializeField] private int ISO = 200;

        [Range(0.001f, 10000f)]
        [SerializeField] private float shutterSpeed = 125;

        [Range(0.7f, 32f)]
        [SerializeField] private float aperture = 16;

        #endregion

        #region [Distortion Correction]

        [Header("Lens Distortion Correction")]
        [SerializeField] private bool distortionCorrection = false;

        #endregion

        #region [Additonal Effect]

        [Header("Additonal Effect")]
        [SerializeField] private bool bloomEffect = false;
        [Range(0f, 1f)]
        [SerializeField] private float bloomValue = 0f;

        [SerializeField] private bool chromaticAberration = false;
        [Range(0f, 1f)]
        [SerializeField] private float chromaticAberrationValue = 0f;

        [SerializeField] private bool depthOfField = false;
        [Range(0.1f, 100f)]
        [SerializeField] private float focusDistance = 0.1f;

        [SerializeField] private bool motionBlur = false;

        #endregion

        #region [Effects Component]

        private ColorAdjustments colorAdjustmentsComponent = default;
        private Exposure exposureComponent = default;
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

            if(!profile.TryGet<Exposure>(out exposureComponent))
            {
                Debug.LogWarning("The required Exposure property is not found in camera volume profile.");
            }

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

            // exposure settings
            ApplyExposureSettings();

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

        private void ApplyExposureSettings()
        {
            camera.GetComponent<HDAdditionalCameraData>().physicalParameters.iso = ISO;
            camera.GetComponent<HDAdditionalCameraData>().physicalParameters.shutterSpeed = 1f/shutterSpeed;
            camera.GetComponent<HDAdditionalCameraData>().physicalParameters.aperture = aperture;
        }
    
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
            }
        }

        private void ApplyMotionBlur()
        {
            if(motionBlurComponent != null)
            {
                motionBlurComponent.active = motionBlur;
                motionBlurComponent.intensity.overrideState = motionBlur;

                float shutterSpeed = camera.GetComponent<HDAdditionalCameraData>().physicalParameters.shutterSpeed;
                motionBlurComponent.intensity.value = 2f * shutterSpeed / Time.fixedDeltaTime;
            }
        }
    }

}