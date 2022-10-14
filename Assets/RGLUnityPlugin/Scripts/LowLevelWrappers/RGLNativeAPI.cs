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
using System.Runtime.InteropServices;
using UnityEngine;

namespace RGLUnityPlugin
{
    public class RGLException : Exception
    {
        public RGLException()
        {
        }

        public RGLException(string message)
            : base(message)
        {
        }

        public RGLException(string message, Exception inner)
            : base(message, inner)
        {
        }
    }

    public enum RGLFormat
    {
        RglFormatInvalid = 0,
        RglFormatXYZ = 1,
        RglFormatCount
    };

    public enum RGLFormatE2E
    {
        RglFormatE2EInvalid = RGLFormat.RglFormatCount,
        RglFormatE2EPcl12,
        RglFormatE2EPcl24,
        RglFormatE2EPcl48,
        RglFormatE2ECount,
    };

    public static class RGLNativeAPI
    {
        public static void CheckVersion()
        {
            int expectedMajor = 0;
            int expectedMinor = 8;
            CheckErr(rgl_get_version_info(out var major, out var minor, out var patch));
            if (major != expectedMajor || minor < expectedMinor)
            {
                throw new RGLException("RGL version mismatch");
            }

            Debug.Log($"RGL Version: {major}.{minor}.{patch}");
        }

        public static void CheckErr(int status)
        {
            if (status == 0)
            {
                return;
            }

            rgl_get_last_error_string(out var errStrPtr);
            string errStr = Marshal.PtrToStringAuto(errStrPtr);
            throw new RGLException(errStr);
        }

        // Public RGL API
        [DllImport("RobotecGPULidar")]
        public static extern int rgl_get_version_info(out int major, out int minor, out int patch);

        [DllImport("RobotecGPULidar")]
        public static extern void rgl_get_last_error_string(out IntPtr error_string);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_mesh_create(out IntPtr mesh, IntPtr vertices, int vertexCount, IntPtr indices,
            int indexCount);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_mesh_destroy(IntPtr mesh);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_mesh_set_vertices(IntPtr mesh, IntPtr vertices, int vertex_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_entity_create(out IntPtr entity, IntPtr scene, IntPtr mesh);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_entity_destroy(IntPtr entity);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_entity_set_pose(IntPtr entity, IntPtr local_to_world_tf);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_create(out IntPtr lidar, IntPtr ray_transforms, int ray_transforms_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_set_range(IntPtr lidar, float range);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_destroy(IntPtr lidar);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_set_pose(IntPtr lidar, IntPtr local_to_world_tf);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_raytrace_async(IntPtr scene, IntPtr lidar);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_get_output_size(IntPtr lidar, out int size);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_get_output_data(IntPtr lidar, int format, IntPtr data);

        // E2E-specific extensions
        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_set_ring_indices(IntPtr lidar, IntPtr ring_ids, int ring_ids_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_set_gaussian_noise_params(IntPtr lidar, int angular_noise_type,
            float angular_noise_stddev, float angular_noise_mean, float distance_noise_stddev_base,
            float distance_noise_stddev_rise_per_meter, float distance_noise_mean);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_lidar_set_post_raycast_transform(IntPtr lidar, IntPtr transform);

    }
}