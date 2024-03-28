using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace AWSIM
{

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
        
        #endregion

        [Header("Exposure Settings")]
        [SerializeField] private CameraExposureMode exposureMode = CameraExposureMode.MANUAL;
        [SerializeField] private int ISO = 200;
        [SerializeField] private float shutterSpeed = 125;
        [SerializeField] private float aperture = 16;

        
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

            }

            if(!profile.TryGet<Exposure>(out exposureComponent))
            {

            }


            if(!profile.TryGet<Bloom>(out bloomComponent))
            {
  
            }

            if(!profile.TryGet<ChromaticAberration>(out chromaticAberrationComponent))
            {
  
            }
            
            if(!profile.TryGet<DepthOfField>(out depthOfFieldComponent))
            {
                
            }

            if(!profile.TryGet<MotionBlur>(out motionBlurComponent))
            {
  
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

            // bloom effect
            ApplyBloom();

            // chromatic aberration
            ApplyChromaticAberration();

            // depth of field
            ApplyDepthOfField();

            // motion blur
            ApplyMotionBlur();


            camera.gameObject.SetActive(cameraObjectActive);
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
