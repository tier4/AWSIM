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
    public static class RGLNativeAPI
    {
        // Public RGL API
        [DllImport("RobotecGPULidar")]
        public static extern int rgl_get_version_info(out int major, out int minor, out int patch);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_configure_logging(RGLLogLevel log_level, [MarshalAs(UnmanagedType.LPStr)] string log_file_path, bool use_stdout);

        [DllImport("RobotecGPULidar")]
        public static extern void rgl_get_last_error_string(out IntPtr error_string);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_mesh_create(out IntPtr mesh, IntPtr vertices, int vertexCount, IntPtr indices, int indexCount);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_mesh_destroy(IntPtr mesh);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_mesh_update_vertices(IntPtr mesh, IntPtr vertices, int vertex_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_entity_create(out IntPtr entity, IntPtr scene, IntPtr mesh);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_entity_destroy(IntPtr entity);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_entity_set_pose(IntPtr entity, IntPtr local_to_world_tf);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_rays_from_mat3x4f(ref IntPtr node, IntPtr rays, int ray_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_rays_set_ring_ids(ref IntPtr node, IntPtr ring_ids, int ring_ids_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_rays_transform(ref IntPtr node, IntPtr transform);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_transform(ref IntPtr node, IntPtr transform);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_raytrace(ref IntPtr node, IntPtr scene, float range);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_format(ref IntPtr node, IntPtr fields, int field_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_yield(ref IntPtr node, IntPtr fields, int field_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_compact(ref IntPtr node);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_downsample(ref IntPtr node, float leaf_size_x, float leaf_size_y, float leaf_size_z);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_write_pcd_file(ref IntPtr node, [MarshalAs(UnmanagedType.LPStr)] string file_path);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_run(IntPtr node);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_destroy(IntPtr node);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_get_result_size(IntPtr node, RGLField field, out Int64 outCount, out Int64 outSizeOf);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_get_result_data(IntPtr node, RGLField field, IntPtr data);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_node_set_active(IntPtr node, bool active);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_node_add_child(IntPtr parent, IntPtr child);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_node_remove_child(IntPtr parent, IntPtr child);


        public static void CheckVersion()
        {
            int expectedMajor = 0;
            int expectedMinor = 11;
            CheckErr(rgl_get_version_info(out var major, out var minor, out var patch));
            if (major != expectedMajor || minor < expectedMinor)
            {
                throw new RGLException($"RGL version mismatch. Expected: {expectedMajor}.>={expectedMinor}.x, but found {major}.{minor}.{patch}.");
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
            string errStr = Marshal.PtrToStringAnsi(errStrPtr);
            throw new RGLException(errStr);
        }

        public static void ConfigureLogging(RGLLogLevel logLevel, string path)
        {
            CheckErr(rgl_configure_logging(logLevel, path, false));
        }

        public static float[] IntoMat3x4f(Matrix4x4[] mats)
        {
            var matFloats = new float[mats.Length * 12];

            for (int i = 0; i < mats.Length; ++i)
            {
                for (int row = 0; row < 3; row++)
                {
                    for (int col = 0; col < 4; col++)
                    {
                        int idx = 12 * i + 4 * row + col;
                        matFloats[idx] = mats[i][row, col];
                    }
                }
            }

            return matFloats;
        }

        public static float[] IntoMat3x4f(Matrix4x4 mat)
        {
            return IntoMat3x4f(new[]{mat});
        }

        public static void NodeRaysFromMat3x4f(ref IntPtr node, Matrix4x4[] rays)
        {
            var rayFloats = IntoMat3x4f(rays);

            unsafe
            {
                fixed (float* rayFloatsPtr = rayFloats)
                {
                    CheckErr(rgl_node_rays_from_mat3x4f(ref node, (IntPtr) rayFloatsPtr, rays.Length));
                }
            }
        }

        public static void NodeRaysSetRingIds(ref IntPtr node, int[] ringIds)
        {
            unsafe
            {
                fixed (int* ringIdsPtr = ringIds)
                {
                    CheckErr(rgl_node_rays_set_ring_ids(ref node, (IntPtr) ringIdsPtr, ringIds.Length));
                }
            }
        }

        public static void NodeRaysTransform(ref IntPtr node, Matrix4x4 transform)
        {
            var tfFloats = IntoMat3x4f(transform);
            unsafe
            {
                fixed (float* tfFloatsPtr = tfFloats)
                {
                    CheckErr(rgl_node_rays_transform(ref node, (IntPtr) tfFloatsPtr));
                }
            }
        }

        public static void NodePointsTransform(ref IntPtr node, Matrix4x4 transform)
        {
            var tfFloats = IntoMat3x4f(transform);
            unsafe
            {
                fixed (float* tfFloatsPtr = tfFloats)
                {
                    CheckErr(rgl_node_points_transform(ref node, (IntPtr) tfFloatsPtr));
                }
            }
        }

        public static void NodeRaytrace(ref IntPtr node, float range)
        {
            CheckErr(rgl_node_raytrace(ref node, IntPtr.Zero, range));
        }

        public static void NodePointsFormat(ref IntPtr node, RGLField[] fields)
        {
            unsafe
            {
                fixed (RGLField* fieldsPtr = fields)
                {
                    CheckErr(rgl_node_points_format(ref node, (IntPtr) fieldsPtr, fields.Length));
                }
            }
        }

        public static void NodePointsYield(ref IntPtr node, RGLField[] fields)
        {
            unsafe
            {
                fixed (RGLField* fieldsPtr = fields)
                {
                    CheckErr(rgl_node_points_yield(ref node, (IntPtr) fieldsPtr, fields.Length));
                }
            }
        }

        public static void NodePointsCompact(ref IntPtr node)
        {
            CheckErr(rgl_node_points_compact(ref node));
        }

        public static void NodePointsDownSample(ref IntPtr node, Vector3 leafDims)
        {
            CheckErr(rgl_node_points_downsample(ref node, leafDims.x, leafDims.y, leafDims.z));
        }

        public static void NodePointsWritePCDFile(ref IntPtr node, string path)
        {
            CheckErr(rgl_node_points_write_pcd_file(ref node, path));
        }

        public static void GraphRun(IntPtr node)
        {
            CheckErr(rgl_graph_run(node));
        }

        public static int GraphGetResult<T>(IntPtr node, RGLField field, ref T[] data, int expectedPointSize) where T : unmanaged
        {
            Int64 pointCount = 0;
            Int64 pointSize = 0;
            CheckErr(rgl_graph_get_result_size(node, field, out pointCount, out pointSize));
            unsafe
            {
                if (pointSize != expectedPointSize)
                {
                    throw new RGLException($"Point size mismatch, requested {expectedPointSize}, but got {pointSize}");
                }
                if (pointCount == 0)
                {
                    return 0;
                }
                if (data.Length * sizeof(T) < pointCount * pointSize)
                {
                    var newSize = pointCount;
                    if (typeof(T) == typeof(byte))
                    {
                        newSize *= pointSize;
                    }
                    data = new T[newSize];
                }
                fixed (T* dataPtr = data)
                {
                    CheckErr(rgl_graph_get_result_data(node, field, (IntPtr) dataPtr));
                }
                return (int) pointCount;
            }
        }

        public static void GraphNodeAddChild(IntPtr parent, IntPtr child)
        {
            CheckErr(rgl_graph_node_add_child(parent, child));
        }

        public static void GraphNodeRemoveChild(IntPtr parent, IntPtr child)
        {
            CheckErr(rgl_graph_node_remove_child(parent, child));
        }

        public static void GraphDestroy(IntPtr node)
        {
            CheckErr(rgl_graph_destroy(node));
        }
    }
}
