using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace AWSIM
{
    /// <summary>
    /// Camera Sensor.
    /// Apply OpenCV distortion and encode to bgr8 format. Use ComputeShader.
    /// </summary>
    public class CameraSensor : MonoBehaviour
    {
        /// <summary>
        /// Camera Parameters
        /// fx, fy = focal length.
        /// cx, cy = camera principal point.
        /// k1, k2, k3, p1, p2 = intrinsic camera parameters.
        /// </summary>
        /// <href>https://docs.opencv.org/4.5.1/dc/dbb/tutorial_py_calibration.html</href>
        [System.Serializable]
        public class CameraParameters
        {
            /// <summary>
            /// Image height.
            /// </summary>
            [Range(256, 2048)] public int width = 1280;

            /// <summary>
            /// Image width.
            /// </summary>
            [Range(256, 2048)] public int height = 720;

            /// <summary>
            /// Focal lengths in pixels
            /// </summary>
            [HideInInspector] public float fx = 0;
            [HideInInspector] public float fy = 0;

            /// <summary>
            /// Principal point in pixels
            /// </summary>
            [HideInInspector] public float cx;
            [HideInInspector] public float cy;

            /// <summary>
            /// Camera distortion coefficients.
            /// For "plumb_bob" model, there are 5 parameters are: (k1, k2, p1, p2, k3).
            /// </summary>
            [Range(-1f, 1f)] public float k1;
            [Range(-1f, 1f)] public float k2;
            [Range(-0.5f, 0.5f)] public float p1;
            [Range(-0.5f, 0.5f)] public float p2;
            [Range(-1f, 1f)] public float k3;

            /// <summary>
            /// Get distortion parameters for "plumb bob" model.
            /// </summary>
            public double[] getDistortionParameters()
            {
                return new double[] { k1, k2, p1, p2, k3 };
            }

            /// <summary>
            /// Get intrinsic camera matrix.
            ///
            /// It projects 3D points in the camera coordinate system to 2D pixel
            /// using focal lengths and principal point.
            ///     [fx  0 cx]
            /// K = [ 0 fy cy]
            ///     [ 0  0  1]
            /// </summary>
            public double[] getCameraMatrix()
            {
                return new double[] {
                    fx, 0, cx,
                    0, fy, cy,
                    0, 0, 1};
            }

            /// <summary>
            /// Get projection matrix.
            ///
            /// For monocular camera Tx = Ty = 0, and P[1:3,1:3] = K
            ///     [fx'  0  cx' Tx]
            /// P = [ 0  fy' cy' Ty]
            ///     [ 0   0   1   0]
            /// </summary>
            public double[] getProjectionMatrix()
            {
                return new double[] {
                    fx, 0, cx, 0,
                    0, fy, cy, 0,
                    0, 0, 1, 0};
            }
        }

        /// <summary>
        /// This data is output from the CameraSensor.
        /// </summary>
        public class OutputData
        {
            /// <summary>
            /// Buffer with image data.
            /// </summary>
            public byte[] imageDataBuffer;

            /// <summary>
            /// Set of the camera parameters.
            /// </summary>
            public CameraParameters cameraParameters;
        }

        [System.Serializable]
        public class ImageOnGui
        {
            /// <summary>
            /// If camera image should be show on GUI
            /// </summary>
            public bool show = true;

            [Range(1, 50)] public uint scale = 1;

            [Range(0, 2048)] public uint xAxis = 0;
            [Range(0, 2048)] public uint yAxis = 0;
        }

        [SerializeField] ImageOnGui imageOnGui = new ImageOnGui();

        [SerializeField] CameraParameters cameraParameters;

        /// <summary>
        /// Delegate used in callbacks.
        /// </summary>
        /// <param name="outputData">Data output for each hz</param>
        public delegate void OnOutputDataDelegate(OutputData outputData);

        /// <summary>
        /// Called each time when data is ready.
        /// </summary>
        public OnOutputDataDelegate OnOutputData;

        /// <summary>
        /// Unity camera object.
        /// </summary>
        [SerializeField] Camera cameraObject;

        RenderTexture targetRenderTexture;
        RenderTexture distortedRenderTexture;

        [SerializeField] ComputeShader distortionShader;
        [SerializeField] ComputeShader rosImageShader;

        int shaderKernelIdx = -1;
        int rosShaderKernelIdx = -1;
        ComputeBuffer computeBuffer;

        OutputData outputData = new OutputData();

        private enum FocalLengthName
        {
            Fx,
            Fy
        }

        private int distortionShaderGroupSizeX;
        private int distortionShaderGroupSizeY;
        private int rosImageShaderGroupSizeX;

        private int bytesPerPixel = 3;

        void Start()
        {
            if (cameraObject == null)
            {
                throw new MissingComponentException("No active Camera component found in GameObject.");
            }

            if (distortionShader == null)
            {
                throw new MissingComponentException("No distortion shader specified.");
            }

            if (rosImageShader == null)
            {
                throw new MissingComponentException("No ros image shader specified.");
            }

            if ((cameraParameters.width * cameraParameters.height) % sizeof(uint) != 0)
            {
                throw new ArgumentException($"Image size {cameraParameters.width} x {cameraParameters.height} should be multiply of {sizeof(uint)}");
            }

            shaderKernelIdx = distortionShader.FindKernel("DistortTexture");
            rosShaderKernelIdx = rosImageShader.FindKernel("RosImageShaderKernel");

            // Set camera parameters
            cameraObject.usePhysicalProperties = true;
            UpdateCameraParameters();
            UpdateRenderTexture();
            ConfigureDistortionShaderBuffers();
            distortionShader.GetKernelThreadGroupSizes(shaderKernelIdx,
                out var distortionShaderThreadsPerGroupX, out var distortionShaderthreadsPerGroupY, out _);
            rosImageShader.GetKernelThreadGroupSizes(rosShaderKernelIdx,
                out var rosImageShaderThreadsPerGroupX, out _ , out _);

            distortionShaderGroupSizeX = ((distortedRenderTexture.width + (int)distortionShaderThreadsPerGroupX - 1) / (int)distortionShaderThreadsPerGroupX);
            distortionShaderGroupSizeY = ((distortedRenderTexture.height + (int)distortionShaderthreadsPerGroupY - 1) / (int)distortionShaderthreadsPerGroupY);
            rosImageShaderGroupSizeX = (((cameraParameters.width * cameraParameters.height) * sizeof(uint)) / ((int)rosImageShaderThreadsPerGroupX * sizeof(uint)));
        }

        public void DoRender()
        {
            // Reander Unity Camera
            cameraObject.Render();

            // Set data to shader
            UpdateShaderParameters();
            distortionShader.SetTexture(shaderKernelIdx, "_InputTexture", targetRenderTexture);
            distortionShader.SetTexture(shaderKernelIdx, "_DistortedTexture", distortedRenderTexture);
            distortionShader.Dispatch(shaderKernelIdx, distortionShaderGroupSizeX, distortionShaderGroupSizeY, 1);
            rosImageShader.SetTexture(rosShaderKernelIdx, "_InputTexture", distortedRenderTexture);
            rosImageShader.SetBuffer(rosShaderKernelIdx, "_RosImageBuffer", computeBuffer);
            rosImageShader.Dispatch(rosShaderKernelIdx, rosImageShaderGroupSizeX, 1, 1);

            // Get data from shader
            AsyncGPUReadback.Request(computeBuffer, OnGPUReadbackRequest);

            // Callback called once the AsyncGPUReadback request is fullfield.
            void OnGPUReadbackRequest(AsyncGPUReadbackRequest request)
            {
                if (request.hasError)
                {
                    Debug.LogWarning("AsyncGPUReadback error");
                    return;
                }
                request.GetData<byte>().CopyTo(outputData.imageDataBuffer);
            }

            // Update output data.
            outputData.cameraParameters = cameraParameters;

            // Call registered callback.
            OnOutputData.Invoke(outputData);
        }

        private void OnDestroy()
        {
            computeBuffer.Release();
        }

        private void ConfigureDistortionShaderBuffers()
        {
            // RosImageShader translates Texture2D to ROS image by encoding two pixels color (bgr8 -> 3 bytes) into one uint32 (4 bytes).
            var rosImageBufferSize = (cameraParameters.width * cameraParameters.height * bytesPerPixel) / sizeof(uint);
            if (computeBuffer == null || computeBuffer.count != rosImageBufferSize)
            {
                computeBuffer = new ComputeBuffer(rosImageBufferSize, sizeof(uint));
                outputData.imageDataBuffer = new byte[cameraParameters.width * cameraParameters.height * bytesPerPixel];
            }
        }

        void OnGUI()
        {
            if (imageOnGui.show)
            {
                GUI.DrawTexture(new Rect(imageOnGui.xAxis, imageOnGui.yAxis,
                    distortedRenderTexture.width / imageOnGui.scale, distortedRenderTexture.height / imageOnGui.scale),
                    distortedRenderTexture);
            }
        }

        private bool FloatEqual(float value1, float value2, float epsilon = 0.001f)
        {
            return Math.Abs(value1 - value2) < epsilon;
        }

        private void VerifyFocalLengthInPixels(ref float focalLengthInPixels, float imageSize, float sensorSize, FocalLengthName focalLengthInPixelsName)
        {
            var computedFocalLengthInPixels = imageSize / sensorSize * cameraObject.focalLength;
            if (focalLengthInPixels == 0.0)
            {
                focalLengthInPixels = computedFocalLengthInPixels;
            }
            else if (!FloatEqual(focalLengthInPixels, computedFocalLengthInPixels))
            {
                Debug.LogWarning("The <" + focalLengthInPixelsName + "> [" + focalLengthInPixels +
                "] you have provided for camera is inconsistent with specified <imageSize> [" + imageSize +
                "] and sensorSize [" + sensorSize + "]. Please double check to see that " + focalLengthInPixelsName + " = imageSize " +
                " / sensorSize * focalLength. The expected " + focalLengthInPixelsName + " value is [" + computedFocalLengthInPixels +
                "], please update your camera model description accordingly. Set " + focalLengthInPixelsName + " to 0 to calculate it automatically");
            }
        }

        private void UpdateRenderTexture()
        {
            targetRenderTexture = new RenderTexture(
                cameraParameters.width, cameraParameters.height, 32, RenderTextureFormat.BGRA32);

            distortedRenderTexture = new RenderTexture(
                cameraParameters.width, cameraParameters.height, 24, RenderTextureFormat.BGRA32, RenderTextureReadWrite.sRGB)
            {
                dimension = TextureDimension.Tex2D,
                antiAliasing = 1,
                useMipMap = false,
                useDynamicScale = false,
                wrapMode = TextureWrapMode.Clamp,
                filterMode = FilterMode.Bilinear,
                enableRandomWrite = true
            };

            distortedRenderTexture.Create();

            cameraObject.targetTexture = targetRenderTexture;
        }

        private void UpdateShaderParameters()
        {
            distortionShader.SetInt("_width", cameraParameters.width);
            distortionShader.SetInt("_height", cameraParameters.height);
            distortionShader.SetFloat("_fx", cameraParameters.fx);
            distortionShader.SetFloat("_fy", cameraParameters.fy);
            distortionShader.SetFloat("_cx", cameraParameters.cx);
            distortionShader.SetFloat("_cy", cameraParameters.cy);
            distortionShader.SetFloat("_k1", -cameraParameters.k1); // TODO: Find out why 'minus' is needed for proper distortion
            distortionShader.SetFloat("_k2", -cameraParameters.k2); // TODO: Find out why 'minus' is needed for proper distortion
            distortionShader.SetFloat("_p1", cameraParameters.p1);
            distortionShader.SetFloat("_p2", -cameraParameters.p2); // TODO: Find out why 'minus' is needed for proper distortion
            distortionShader.SetFloat("_k3", -cameraParameters.k3); // TODO: Find out why 'minus' is needed for proper distortion

            rosImageShader.SetInt("_width", cameraParameters.width);
            rosImageShader.SetInt("_height", cameraParameters.height);
        }

        private void UpdateCameraParameters()
        {
            VerifyFocalLengthInPixels(ref cameraParameters.fx, cameraParameters.width, cameraObject.sensorSize.x, FocalLengthName.Fx);
            VerifyFocalLengthInPixels(ref cameraParameters.fy, cameraParameters.height, cameraObject.sensorSize.y, FocalLengthName.Fy);
            cameraParameters.cx = ((cameraParameters.width + 1) / 2.0f);
            cameraParameters.cy = ((cameraParameters.height + 1) / 2.0f);
        }
    }
}