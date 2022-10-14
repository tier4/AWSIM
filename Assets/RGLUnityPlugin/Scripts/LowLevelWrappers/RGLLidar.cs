// Copyright 2022 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using UnityEngine;

namespace RGLUnityPlugin
{
    /// <summary>
    /// This class provides automatic creation & destruction of the native Lidar object.
    /// </summary>
    public class RGLLidar
    {
        private IntPtr lidar;

        internal RGLLidar(Matrix4x4[] rayPoses, int[] ringIds)
        {
            var rayPoses3x4Floats = new float[rayPoses.Length * 12];

            for (int i = 0; i < rayPoses.Length; ++i)
            {
                for (int row = 0; row < 3; row++)
                {
                    for (int col = 0; col < 4; col++)
                    {
                        int idx = 12 * i + 4 * row + col;
                        rayPoses3x4Floats[idx] = rayPoses[i][row, col];
                    }
                }
            }

            unsafe
            {
                fixed (float* rayPoses3x4FloatsPtr = rayPoses3x4Floats)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_lidar_create(
                            out lidar, (IntPtr) rayPoses3x4FloatsPtr, rayPoses3x4Floats.Length / 12));
                }
            }

            unsafe
            {
                fixed (int* ringIdsPtr = ringIds)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_lidar_set_ring_indices(
                            lidar, (IntPtr) ringIdsPtr, ringIds.Length));
                }
            }

        }

        public void RaytraceAsync(Matrix4x4 lidarToWorld, Matrix4x4 postRaycastTransform, float range)
        {
            var lidarToWorld3x4Floats = new float[12];
            var postRaycastTransform3x4Floats = new float[12];
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 4; col++)
                {
                    lidarToWorld3x4Floats[4 * row + col] = lidarToWorld[row, col];
                    postRaycastTransform3x4Floats[4 * row + col] = postRaycastTransform[row, col];
                }
            }

            RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_lidar_set_range(lidar, range));
            unsafe
            {
                fixed (float* tf = postRaycastTransform3x4Floats, pose = lidarToWorld3x4Floats)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_lidar_set_pose(
                            lidar, (IntPtr) pose));
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_lidar_set_post_raycast_transform(
                            lidar, (IntPtr) tf));
                }
            }

            RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_lidar_raytrace_async(IntPtr.Zero, lidar));
        }

        public void SyncAndDownload(ref int hitCount, ref Vector3[] hits, ref byte[] rosPCL24, ref byte[] rosPCL48)
        {
            RGLNativeAPI.CheckErr(
                RGLNativeAPI.rgl_lidar_get_output_size(lidar, out hitCount));
            unsafe
            {
                fixed (Vector3* dataPtr = hits)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_lidar_get_output_data(lidar, (int) RGLFormat.RglFormatXYZ, (IntPtr) dataPtr));
                }

                fixed (byte* dataPtr = rosPCL24)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_lidar_get_output_data(lidar, (int) RGLFormatE2E.RglFormatE2EPcl24,
                            (IntPtr) dataPtr));
                }

                fixed (byte* dataPtr = rosPCL48)
                {
                    RGLNativeAPI.CheckErr(
                        RGLNativeAPI.rgl_lidar_get_output_data(lidar, (int) RGLFormatE2E.RglFormatE2EPcl48,
                            (IntPtr) dataPtr));
                }
            }
        }

        public void SetGaussianNoiseParamsCtx(LidarNoiseParams param)
        {
            RGLNativeAPI.CheckErr(
                RGLNativeAPI.rgl_lidar_set_gaussian_noise_params(
                    lidar, (int) param.angularNoiseType, Mathf.Deg2Rad * param.angularNoiseStDev,
                    Mathf.Deg2Rad * param.angularNoiseMean,
                    param.distanceNoiseStDevBase, param.distanceNoiseStDevRisePerMeter, param.distanceNoiseMean));
        }

        ~RGLLidar()
        {
            RGLNativeAPI.CheckErr(RGLNativeAPI.rgl_lidar_destroy(lidar));
        }
    }
}