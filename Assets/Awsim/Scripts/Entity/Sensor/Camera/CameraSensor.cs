// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace Awsim.Entity
{
    /// <summary>
    /// Camera sensor capable of using OpenCV camera distortion parameters.
    /// NOTE: The Camera sensor eats computing resources. So, multiple camera sensors are rendered sequentially by the CameraSensorScheduller class.
    /// </summary>
    public class CameraSensor : MonoBehaviour, ICameraSensor
    {
        /// <summary>
        /// Camera parameters (Read only).
        /// </summary>
        public interface IReadOnlyCameraParameters
        {
            /// <summary>
            /// Image width (px).
            /// </summary>
            public int Width { get; }

            /// <summary>
            /// Image height (px).
            /// </summary>
            public int Height { get; }

            /// <summary>
            /// Focal length on x-axis (px).
            /// </summary>
            public float Fx { get; }

            /// <summary>
            /// Focal length on y-axis (px).
            /// </summary>
            public float Fy { get; }

            /// <summary>
            /// Principal point on x-axis (px).
            /// </summary>
            public float Cx { get; }

            /// <summary>
            /// Principal point on y-axis (px).
            /// </summary>
            public float Cy { get; }

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's k1 parameter.
            /// </summary>
            public float K1 { get; }

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's k2 parameter.
            /// </summary>
            public float K2 { get; }

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's p1 parameter.
            /// </summary>
            public float P1 { get; }

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's p2 parameter.
            /// </summary>
            public float P2 { get; }

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's k3 parameter.
            /// </summary>
            public float K3 { get; }

            /// <summary>
            /// Get distortion parameters for plumb bob model.
            /// 
            /// Distrion Coefficients = { k1, k2, p1, p2, k3 }
            /// </summary>
            /// <returns>Distorion coefficients.</returns>
            public double[] GetDistortionCoefficients();

            /// <summary>
            /// Get intrinsic camera matrix.
            ///
            /// It projects 3D points in the camera coordinate system to 2D pixel
            /// using focal lengths and principal point.
            ///     [fx  0 cx]
            /// K = [ 0 fy cy]
            ///     [ 0  0  1]
            /// </summary>
            /// <returns>Intrinsic camera matrix.</returns>
            public double[] GetCameraMatrix();

            /// <summary>
            /// Get projection matrix.
            ///
            /// For monocular camera Tx = Ty = 0, and P[1:3,1:3] = K
            ///     [fx'  0  cx' Tx]
            /// P = [ 0  fy' cy' Ty]
            ///     [ 0   0   1   0]
            /// </summary>
            /// <returns>Projection camera matrix</returns>
            public double[] GetProjectionMatrix();
        }

        /// <summary>
        /// Camera parameters.
        /// </summary>
        public class CameraParameters : IReadOnlyCameraParameters
        {
            /// <summary>
            /// Image width (px).
            /// </summary>
            public int Width { get; set; } = 0;

            /// <summary>
            /// Image height (px).
            /// </summary>
            public int Height { get; set; } = 0;

            /// <summary>
            /// Focal length on x-axis (px).
            /// </summary>
            public float Fx { get; set; } = 0;

            /// <summary>
            /// Focal length on y-axis (px).
            /// </summary>
            public float Fy { get; set; } = 0;

            /// <summary>
            /// Principal point on x-axis (px).
            /// </summary>
            public float Cx { get; set; } = 0;

            /// <summary>
            /// Principal point on y-axis (px).
            /// </summary>
            public float Cy { get; set; } = 0;

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's k1 parameter.
            /// </summary>
            public float K1 { get; set; } = 0;

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's k2 parameter.
            /// </summary>
            public float K2 { get; set; } = 0;

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's p1 parameter.
            /// </summary>
            public float P1 { get; set; } = 0;

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's p2 parameter.
            /// </summary>
            public float P2 { get; set; } = 0;

            /// <summary>
            /// Distortion coefficient. For plumb_bob models's k3 parameter.
            /// </summary>
            public float K3 { get; set; } = 0;


            /// <summary>
            /// Get distortion parameters for plumb bob model.
            /// 
            /// Distrion Coefficients = { k1, k2, p1, p2, k3 }
            /// </summary>
            /// <returns>Distorion coefficients.</returns>
            public double[] GetDistortionCoefficients()
            {
                var d = _distortionParameters;

                d[0] = K1;
                d[1] = K2;
                d[2] = P1;
                d[3] = P2;
                d[4] = K3;

                return d;
            }

            /// <summary>
            /// Get intrinsic camera matrix.
            ///
            /// It projects 3D points in the camera coordinate system to 2D pixel
            /// using focal lengths and principal point.
            ///     [fx] [ 0] [cx]         [0] [1] [2]
            /// K = [ 0] [fy] [cy] index = [3] [4] [5]
            ///     [ 0] [ 0] [ 1]         [6] [7] [8]
            /// </summary>
            /// <returns>Intrinsic camera matrix.</returns>
            public double[] GetCameraMatrix()
            {
                var m = _cameraMatrix;

                m[0] = Fx;
                m[2] = Cx;
                m[4] = Fy;
                m[5] = Cy;

                return m;
            }

            /// <summary>
            /// Get projection matrix.
            ///
            /// For monocular camera Tx = Ty = 0, and P[1:3,1:3] = K
            ///     [fx'] [  0] [cx'] [ Tx]         [ 0] [ 1] [ 2] [ 3]
            /// P = [  0] [fy'] [cy'] [ Ty] index = [ 4] [ 5] [ 6] [ 7]// 
            ///     [  0] [  0] [  1] [  0]         [ 8] [ 9] [10] [11]
            /// </summary>
            /// <returns>Projection camera matrix</returns>
            public double[] GetProjectionMatrix()
            {
                var m = _projectionMatrix;

                m[0] = Fx;
                m[2] = Cx;
                m[5] = Fy;
                m[6] = Cy;

                return m;
            }

            double[] _distortionParameters = new double[5] { 0, 0, 0, 0, 0 };
            double[] _cameraMatrix = new double[9]
            {
                0, 0, 0,
                0, 0, 0,
                0, 0, 1,
            };
            double[] _projectionMatrix = new double[12]
                                        {
                                            0, 0, 0, 0,
                                            0, 0, 0, 0,
                                            0, 0, 1, 0,
                                        };
        }

        /// <summary>
        /// Camera sensor output data (Read only).
        /// </summary>
        public interface IReadOnlyOutputData
        {
            /// <summary>
            /// Camera parameters.
            /// </summary>
            public IReadOnlyCameraParameters CameraParameters { get; }

            /// <summary>
            /// Outputted render texture.
            /// </summary>
            public RenderTexture OutputRenderTexture { get; }
        }

        /// <summary>
        /// Camera sensor output data.
        /// </summary>
        public class OutputData : IReadOnlyOutputData
        {
            /// <summary>
            /// Camera parameters.
            /// </summary>
            public IReadOnlyCameraParameters CameraParameters { get; set; }

            /// <summary>
            /// Outputted render texture.
            /// </summary>
            public RenderTexture OutputRenderTexture { get; set; }
        }

        /// <summary>
        /// Callback action to be fired on Output. Invoked according to the OutputHz cycle.
        /// </summary>
        public Action<IReadOnlyOutputData> OnOutput { get; set; } = null;

        [Header("Camera settings")]
        [SerializeField] int _width = 1920;
        [SerializeField] int _height = 1080;
        float _fx = 0f;
        float _fy = 0f;
        float _cx = 0f;
        float _cy = 0f;
        [SerializeField] float _k1 = 0f;
        [SerializeField] float _k2 = 0f;
        [SerializeField] float _p1 = 0f;
        [SerializeField] float _p2 = 0f;
        [SerializeField] float _k3 = 0f;
        [SerializeField] UnityEngine.Camera _camera;
        [SerializeField] bool _enableLensDistortionCorrection = false;
        [SerializeField, Range(0.0f, 1.0f)] float _sharpeningStrength = 0.0f;

        [Header("Compute shaders")]
        [SerializeField] ComputeShader _distortionShader = null;
        [SerializeField] ComputeShader _distortionCorrectionShader = null;
        [SerializeField] ComputeShader _sharpenShader = null;

        RenderTexture _targetRenderTexture = null;
        RenderTexture _distortedRenderTexture = null;
        RenderTexture _distortionCorrectionRenderTexture = null;
        RenderTexture _sharpenRenderTexture = null;

        int _shaderKernelIdx = -1;
        int _cameraDistortionCorrectionShaderKernelIdx = -1;
        int _sharpenShaderKernelIdx = -1;

        int _distortionShaderGroupSizeX = 0;
        int _distortionShaderGroupSizeY = 0;
        int _distortionCorrectionShaderGroupSizeX = 0;
        int _distortionCorrectionShaderGroupSizeY = 0;
        int _sharpenShaderGroupSizeX = 0;
        int _sharpenShaderGroupSizeY = 0;

        const string _sharpenShaderKernel = "SharpenTexture";
        const string _distortionShaderKernel = "DistortTexture";
        const string _distortionCorrectionShaderKernel = "CameraDistortionCorrection";

        OutputData _outputData = null;
        RenderTexture _outputRenderTexture = null;
        CameraParameters _outputCameraParameters = null;

        /// <summary>
        /// Initialization of camera sensor.
        /// </summary>
        public void Initialize()
        {
            // Initialize output data.
            _outputData = new OutputData();
            _outputCameraParameters = new CameraParameters();
            _outputData.OutputRenderTexture = _outputRenderTexture;
            _outputData.CameraParameters = _outputCameraParameters;

            // Find shader kernels.
            _sharpenShaderKernelIdx = _sharpenShader.FindKernel(_sharpenShaderKernel);
            _shaderKernelIdx = _distortionShader.FindKernel(_distortionShaderKernel);
            _cameraDistortionCorrectionShaderKernelIdx = _distortionCorrectionShader.FindKernel(_distortionCorrectionShaderKernel);

            // Set camera parameters.
            VerifyFocalLengthInPixels(ref _fx, _width, _camera.sensorSize.x, _camera.focalLength, "Fx");
            VerifyFocalLengthInPixels(ref _fy, _height, _camera.sensorSize.y, _camera.focalLength, "Fy");
            _cx = (_width + 1) / 2.0f;
            _cy = (_height + 1) / 2.0f;
            UpdateRenderTexture();

            _distortionShader.GetKernelThreadGroupSizes(_shaderKernelIdx,
                out var distortionShaderThreadsPerGroupX, out var distortionShaderThreadsPerGroupY, out _);
            _distortionCorrectionShader.GetKernelThreadGroupSizes(_cameraDistortionCorrectionShaderKernelIdx,
                out var distortionCorrectionShaderThreadsPerGroupX, out var distortionCorrectionShaderThreadsPerGroupY, out _);
            _sharpenShader.GetKernelThreadGroupSizes(_sharpenShaderKernelIdx,
                out var sharpenShaderThreadsPerGroupX, out var sharpenShaderThreadsPerGroupY, out _);

            _distortionShaderGroupSizeX = (_distortedRenderTexture.width + (int)distortionShaderThreadsPerGroupX - 1) / (int)distortionShaderThreadsPerGroupX;
            _distortionShaderGroupSizeY = (_distortedRenderTexture.height + (int)distortionShaderThreadsPerGroupY - 1) / (int)distortionShaderThreadsPerGroupY;
            _distortionCorrectionShaderGroupSizeX = (_distortedRenderTexture.width + (int)distortionCorrectionShaderThreadsPerGroupX - 1) / (int)distortionCorrectionShaderThreadsPerGroupX;
            _distortionCorrectionShaderGroupSizeY = (_distortedRenderTexture.height + (int)distortionCorrectionShaderThreadsPerGroupY - 1) / (int)distortionCorrectionShaderThreadsPerGroupY;
            _sharpenShaderGroupSizeX = (_sharpenRenderTexture.width + (int)sharpenShaderThreadsPerGroupX - 1) / (int)sharpenShaderThreadsPerGroupX;
            _sharpenShaderGroupSizeY = (_sharpenRenderTexture.height + (int)sharpenShaderThreadsPerGroupY - 1) / (int)sharpenShaderThreadsPerGroupY;

            // Inner methods.
            static void VerifyFocalLengthInPixels(ref float focalLengthInPixels, float imageSize, float sensorSize, float focalLength, string focalLengthInPixelsName)
            {
                var computedFocalLengthInPixels = imageSize / sensorSize * focalLength;

                const float epsilon = 0.001f;
                var overEpsilon = Mathf.Abs(focalLengthInPixels - computedFocalLengthInPixels) >= epsilon;
                if (focalLengthInPixels == 0.0f)
                {
                    focalLengthInPixels = computedFocalLengthInPixels;
                }
                else if (overEpsilon)
                {
                    Debug.LogWarning("The <" + focalLengthInPixelsName + "> [" + focalLengthInPixels +
                    "] you have provided for camera is inconsistent with specified <imageSize> [" + imageSize +
                    "] and sensorSize [" + sensorSize + "]. Please double check to see that " + focalLengthInPixelsName + " = imageSize " +
                    " / sensorSize * focalLength. The expected " + focalLengthInPixelsName + " value is [" + computedFocalLengthInPixels +
                    "], please update your camera model description accordingly. Set " + focalLengthInPixelsName + " to 0 to calculate it automatically");
                }
            }

            void UpdateRenderTexture()
            {
                _targetRenderTexture = new RenderTexture(_width, _height, 32, RenderTextureFormat.BGRA32);

                _distortedRenderTexture = new RenderTexture(_width, _height, 24, RenderTextureFormat.BGRA32, RenderTextureReadWrite.sRGB)
                {
                    dimension = TextureDimension.Tex2D,
                    antiAliasing = 1,
                    useMipMap = false,
                    useDynamicScale = false,
                    wrapMode = TextureWrapMode.Clamp,
                    filterMode = FilterMode.Bilinear,
                    enableRandomWrite = true
                };
                _distortedRenderTexture.Create();

                _distortionCorrectionRenderTexture = new RenderTexture(_width, _height, 24, RenderTextureFormat.BGRA32, RenderTextureReadWrite.sRGB)
                {
                    dimension = TextureDimension.Tex2D,
                    antiAliasing = 1,
                    useMipMap = false,
                    useDynamicScale = false,
                    wrapMode = TextureWrapMode.Clamp,
                    filterMode = FilterMode.Bilinear,
                    enableRandomWrite = true
                };
                _distortionCorrectionRenderTexture.Create();

                _sharpenRenderTexture = new RenderTexture(_width, _height, 24, RenderTextureFormat.BGRA32, RenderTextureReadWrite.sRGB)
                {
                    dimension = TextureDimension.Tex2D,
                    antiAliasing = 1,
                    useMipMap = false,
                    useDynamicScale = false,
                    wrapMode = TextureWrapMode.Clamp,
                    filterMode = FilterMode.Bilinear,
                    enableRandomWrite = true
                };
                _sharpenRenderTexture.Create();

                _camera.targetTexture = _targetRenderTexture;
            }
        }

        /// <summary>
        /// The camera sensor renders one frame. Outputs the RenderTexture and camera parameters with each shader applied.
        /// </summary>
        public void DoRender()
        {
            // Render unity camera.
            _camera.Render();

            // Shader const.
            const string _width = "_width";
            const string _height = "_height";
            const string _fx = "_fx";
            const string _fy = "_fy";
            const string _cx = "_cx";
            const string _cy = "_cy";
            const string _k1 = "_k1";
            const string _k2 = "_k2";
            const string _p1 = "_p1";
            const string _p2 = "_p2";
            const string _k3 = "_k3";
            const string _sharpeningStrength = "_sharpeningStrength";
            const string _InputTexture = "_InputTexture";
            const string _ResultTexture = "_ResultTexture";
            const string _DistortedTexture = "_DistortedTexture";

            // Set data to shader.
            _distortionShader.SetInt(_width, this._width);
            _distortionShader.SetInt(_height, this._height);
            _distortionShader.SetFloat(_fx, this._fx);
            _distortionShader.SetFloat(_fy, this._fy);
            _distortionShader.SetFloat(_cx, this._cx);
            _distortionShader.SetFloat(_cy, this._cy);
            _distortionShader.SetFloat(_k1, -this._k1);                 // TODO: Find out why 'minus' is needed for proper distortion.
            _distortionShader.SetFloat(_k2, -this._k2);                 // TODO: Find out why 'minus' is needed for proper distortion.
            _distortionShader.SetFloat(_p1, this._p1);
            _distortionShader.SetFloat(_p2, -this._p2);                 // TODO: Find out why 'minus' is needed for proper distortion.
            _distortionShader.SetFloat(_k3, -this._k3);                 // TODO: Find out why 'minus' is needed for proper distortion.
            _distortionCorrectionShader.SetInt(_width, this._width);
            _distortionCorrectionShader.SetInt(_height, this._height);
            _distortionCorrectionShader.SetFloat(_fx, this._fx);
            _distortionCorrectionShader.SetFloat(_fy, this._fy);
            _distortionCorrectionShader.SetFloat(_cx, this._cx);
            _distortionCorrectionShader.SetFloat(_cy, this._cy);
            _distortionCorrectionShader.SetFloat(_k1, -this._k1);       // TODO: Find out why 'minus' is needed for proper distortion.
            _distortionCorrectionShader.SetFloat(_k2, -this._k2);       // TODO: Find out why 'minus' is needed for proper distortion.
            _distortionCorrectionShader.SetFloat(_p1, this._p1);
            _distortionCorrectionShader.SetFloat(_p2, -this._p2);       // TODO: Find out why 'minus' is needed for proper distortion.
            _distortionCorrectionShader.SetFloat(_k3, -this._k3);       // TODO: Find out why 'minus' is needed for proper distortion.
            _sharpenShader.SetFloat(_sharpeningStrength, this._sharpeningStrength);

            // Dispatch sharpen shader.
            _sharpenShader.SetTexture(_sharpenShaderKernelIdx, _InputTexture, _targetRenderTexture);
            _sharpenShader.SetTexture(_sharpenShaderKernelIdx, _ResultTexture, _sharpenRenderTexture);
            _sharpenShader.Dispatch(_sharpenShaderKernelIdx, _sharpenShaderGroupSizeX, _sharpenShaderGroupSizeY, 1);

            // Dispatch distortion shader.
            _distortionShader.SetTexture(_shaderKernelIdx, _InputTexture, _sharpenRenderTexture);
            _distortionShader.SetTexture(_shaderKernelIdx, _DistortedTexture, _distortedRenderTexture);
            _distortionShader.Dispatch(_shaderKernelIdx, _distortionShaderGroupSizeX, _distortionShaderGroupSizeY, 1);

            // Dispatch distortion correction shader.
            if (_enableLensDistortionCorrection)
            {
                _distortionCorrectionShader.SetTexture(_cameraDistortionCorrectionShaderKernelIdx, _InputTexture, _distortedRenderTexture);
                _distortionCorrectionShader.SetTexture(_cameraDistortionCorrectionShaderKernelIdx, _DistortedTexture, _distortionCorrectionRenderTexture);
                _distortionCorrectionShader.Dispatch(_cameraDistortionCorrectionShaderKernelIdx, _distortionCorrectionShaderGroupSizeX, _distortionCorrectionShaderGroupSizeY, 1);

                // Set output render texture.
                _outputData.OutputRenderTexture = _distortionCorrectionRenderTexture;
            }
            else
            {
                // Set output render texture.
                _outputData.OutputRenderTexture = _distortedRenderTexture;
            }

            // Set camera parameters.
            _outputCameraParameters.Width = this._width;
            _outputCameraParameters.Height = this._height;
            _outputCameraParameters.Fy = this._fy;
            _outputCameraParameters.Fx = this._fx;
            _outputCameraParameters.Cx = this._cx;
            _outputCameraParameters.Cy = this._cy;
            _outputCameraParameters.K1 = this._k1;
            _outputCameraParameters.K2 = this._k2;
            _outputCameraParameters.P1 = this._p1;
            _outputCameraParameters.P2 = this._p2;
            _outputCameraParameters.K3 = this._k3;

            // Invoke output action.
            OnOutput?.Invoke(_outputData);
        }
    }
}
