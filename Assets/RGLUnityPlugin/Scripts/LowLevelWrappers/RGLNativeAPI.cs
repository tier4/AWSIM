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
        public static extern int rgl_get_extension_info(RGLExtension extension, out int available);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_configure_logging(RGLLogLevel log_level, [MarshalAs(UnmanagedType.LPStr)] string log_file_path, bool use_stdout);

        [DllImport("RobotecGPULidar")]
        public static extern void rgl_get_last_error_string(out IntPtr error_string);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_mesh_create(out IntPtr mesh, IntPtr vertices, int vertexCount, IntPtr indices, int indexCount);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_mesh_set_texture_coords(IntPtr mesh, IntPtr uvs, int uvCount);

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
        public static extern int rgl_entity_set_id(IntPtr entity, int id);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_entity_set_intensity_texture(IntPtr entity, IntPtr texture);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_texture_create(out IntPtr texture, IntPtr texels, int width, int height);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_texture_destroy(IntPtr texture);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_scene_set_time(IntPtr scene, UInt64 nanoseconds);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_rays_from_mat3x4f(ref IntPtr node, IntPtr rays, int ray_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_rays_set_range(ref IntPtr node, IntPtr ranges, int ranges_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_rays_set_ring_ids(ref IntPtr node, IntPtr ring_ids, int ring_ids_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_rays_set_time_offsets(ref IntPtr node, IntPtr time_offsets, int time_offsets_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_rays_transform(ref IntPtr node, IntPtr transform);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_transform(ref IntPtr node, IntPtr transform);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_raytrace(ref IntPtr node, IntPtr scene);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_raytrace_in_motion(ref IntPtr node, IntPtr scene, IntPtr linear_velocity, IntPtr angular_velocity, bool apply_ray_distortion);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_format(ref IntPtr node, IntPtr fields, int field_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_yield(ref IntPtr node, IntPtr fields, int field_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_compact(ref IntPtr node);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_downsample(ref IntPtr node, float leaf_size_x, float leaf_size_y, float leaf_size_z);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_temporal_merge(ref IntPtr node, IntPtr fields, int field_count);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_ros2_publish_with_qos(
            ref IntPtr node, [MarshalAs(UnmanagedType.LPStr)] string topic_name, [MarshalAs(UnmanagedType.LPStr)] string frame_id,
            RGLQosPolicyReliability qos_reliability, RGLQosPolicyDurability qos_durability, RGLQosPolicyHistory qos_history, int qos_depth);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_udp_publish(
            ref IntPtr node, RGLLidarModel lidar_model, RGLUdpOptions udp_options, [MarshalAs(UnmanagedType.LPStr)] string device_ip,
            [MarshalAs(UnmanagedType.LPStr)] string dest_ip, int dest_port);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_gaussian_noise_angular_ray(ref IntPtr node, float mean, float st_dev, RGLAxis rotation_axis);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_gaussian_noise_angular_hitpoint(ref IntPtr node, float mean, float st_dev, RGLAxis rotation_axis);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_gaussian_noise_distance(ref IntPtr node, float mean, float st_dev, float st_dev_rise_per_meter);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_remove_ground(ref IntPtr node, RGLAxis sensor_up_axis, float ground_angle_threshold,
            float ground_distance_threshold, float ground_filter_distance);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_node_points_radar_postprocess(ref IntPtr node, float distance_separation, float azimuth_separation);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_run(IntPtr node);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_destroy(IntPtr node);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_write_pcd_file(IntPtr node, [MarshalAs(UnmanagedType.LPStr)] string file_path);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_get_result_size(IntPtr node, RGLField field, out Int32 outCount, out Int32 outSizeOf);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_get_result_data(IntPtr node, RGLField field, IntPtr data);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_node_add_child(IntPtr parent, IntPtr child);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_node_remove_child(IntPtr parent, IntPtr child);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_node_set_priority(IntPtr node, Int32 priority);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_graph_node_get_priority(IntPtr node, out Int32 priority);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_tape_record_begin([MarshalAs(UnmanagedType.LPStr)] string path);

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_tape_record_end();

        [DllImport("RobotecGPULidar")]
        public static extern int rgl_tape_record_is_active(out bool isActive);

        static RGLNativeAPI()
        {
            string ros2SourcedCodename = Environment.GetEnvironmentVariable("ROS_DISTRO");
            bool isRos2Sourced = !string.IsNullOrEmpty(ros2SourcedCodename);
            if (isRos2Sourced)
            {
                Debug.LogError(
                    "You should not source ROS2 in 'RobotecGPULidar' standalone build. " +
                    "RGLUnityPlugin might not work correctly."
                );
            }

            try
            {
                CheckVersion();
            }
            catch (DllNotFoundException)
            {
                Debug.LogError($"RobotecGPULidar library cannot be found!");
                return;
            }

            RGLDebugger debugger = UnityEngine.Object.FindObjectOfType<RGLDebugger>(false);
            if (debugger != null)
            {
                ConfigureLogging(debugger.LogLevel, debugger.LogOutputPath);
                if (debugger.ActivateTapeRecord)
                {
                    TapeRecordBegin(debugger.TapeOutputPath);
                }
            }
        }

        public static void CheckVersion()
        {
            int expectedMajor = 0;
            int expectedMinor = 16;
            int expectedPatch = 0;
            CheckErr(rgl_get_version_info(out var major, out var minor, out var patch));
            if (major != expectedMajor || minor < expectedMinor || (minor == expectedMinor && patch < expectedPatch))
            {
                throw new RGLException($"RGL version mismatch. Expected: minimum {expectedMinor}.{expectedPatch} for major {expectedMajor}, " +
                                       $"but found {major}.{minor}.{patch}.");
            }

            Debug.Log($"RGL Version: {major}.{minor}.{patch}");
        }

        public static void CheckErr(int status)
        {
            if (status == (int)RGLStatus.SUCCESS)
            {
                return;
            }

            if (status == (int)RGLStatus.INVALID_STATE)
            {
                foreach (LidarSensor rglLidar in UnityEngine.Object.FindObjectsOfType<LidarSensor>())
                {
                    rglLidar.enabled = false;
                }
                throw new RGLException("A previous unrecoverable error has corrupted RobotecGPULidar's internal state. "
                                       + "Disabling LidarSensor components. The application must be restarted!");
            }

            rgl_get_last_error_string(out var errStrPtr);
            string errStr = Marshal.PtrToStringAnsi(errStrPtr);
            throw new RGLException(errStr);
        }

        public static bool HasExtension(RGLExtension extension)
        {
            CheckErr(rgl_get_extension_info(extension, out var available));
            return available != 0;
        }

        public static void TapeRecordBegin(string path)
        {
            if (string.IsNullOrEmpty(path))
            {
                throw new RGLException("Attempted to start tape recording on empty path.");
            }
            Debug.LogWarning($"Start RGL tape recording on path '{path}'. Two files will be created with .bin and .yaml extensions");
            CheckErr(rgl_tape_record_begin(path));
        }

        public static void TapeRecordEnd()
        {
            Debug.LogWarning("End RGL tape recording");
            CheckErr(rgl_tape_record_end());
        }

        public static bool isTapeRecordActive()
        {
            CheckErr(rgl_tape_record_is_active(out bool isActive));
            return isActive;
        }

        public static void ConfigureLogging(RGLLogLevel logLevel, string path)
        {
            if (string.IsNullOrEmpty(path) && logLevel != RGLLogLevel.OFF)
            {
                throw new RGLException("Attempted to set RGL logging for empty output logging path.");
            }
            CheckErr(rgl_configure_logging(logLevel, path, false));
        }


        public static float[] IntoVec3f(Vector3 vec)
        {
            return new[] {vec.x, vec.y, vec.z};
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

        public static float[] IntoVec2f(Vector2[] vecs)
        {
            var vecFloats = new float[vecs.Length * 2];
            for (int i = 0; i < vecs.Length; ++i)
            {
                vecFloats[2 * i] = vecs[i].x;
                vecFloats[2 * i + 1] = vecs[i].y;
            }
            return vecFloats;
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

        public static void NodeRaysSetRange(ref IntPtr node, Vector2[] ranges)
        {
            var rangesFloats = IntoVec2f(ranges);

            unsafe
            {
                fixed (float* rangesFloatsPtr = rangesFloats)
                {
                    CheckErr(rgl_node_rays_set_range(ref node, (IntPtr) rangesFloatsPtr, ranges.Length));
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

        public static void NodeRaysSetTimeOffsets(ref IntPtr node, float[] offsets)
        {
            unsafe
            {
                fixed (float* offsetsPtr = offsets)
                {
                    CheckErr(rgl_node_rays_set_time_offsets(ref node, (IntPtr) offsetsPtr, offsets.Length));
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

        public static void NodeRaytrace(ref IntPtr node)
        {
            CheckErr(rgl_node_raytrace(ref node, IntPtr.Zero));
        }

        // Raytrace with sensor velocity provided. Needed for velocity distortion feature or radar simulation.
        public static void NodeRaytrace(ref IntPtr node, Vector3 linearVelocity, Vector3 angularVelocity, bool applyRayDistortion)
        {
            var linearVelocityFloats = IntoVec3f(linearVelocity);
            var angularVelocityFloats = IntoVec3f(angularVelocity);

            unsafe
            {
                fixed (float* linearVelocityFloatsPtr = linearVelocityFloats)
                {
                    fixed (float* angularVelocityFloatsPtr = angularVelocityFloats)
                    {
                        CheckErr(rgl_node_raytrace_in_motion(ref node, IntPtr.Zero, (IntPtr) linearVelocityFloatsPtr, (IntPtr) angularVelocityFloatsPtr, applyRayDistortion));
                    }
                }
            }
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

        public static void NodePointsTemporalMerge(ref IntPtr node, RGLField[] fields)
        {
            unsafe
            {
                fixed (RGLField* fieldsPtr = fields)
                {
                    CheckErr(rgl_node_points_temporal_merge(ref node, (IntPtr) fieldsPtr, fields.Length));
                }
            }
        }

        public static void NodePointsRos2PublishWithQos(
            ref IntPtr node, string topicName, string frameId,
            RGLQosPolicyReliability qos_reliability, RGLQosPolicyDurability qos_durability, RGLQosPolicyHistory qos_history, int qos_depth)
        {
            CheckErr(rgl_node_points_ros2_publish_with_qos(ref node, topicName, frameId, qos_reliability, qos_durability, qos_history, qos_depth));
        }

        public static void NodePointsUdpPublish(ref IntPtr node, RGLLidarModel lidarModel, RGLUdpOptions udpOptions, string deviceIp, string destIp, int destPort)
        {
            CheckErr(rgl_node_points_udp_publish(ref node, lidarModel, udpOptions, deviceIp, destIp, destPort));
        }

        public static void NodeGaussianNoiseAngularRay(ref IntPtr node, float mean, float stDev)
        {
            CheckErr(rgl_node_gaussian_noise_angular_ray(ref node, mean, stDev, RGLAxis.RGL_AXIS_Y));
        }

        public static void NodeGaussianNoiseAngularHitpoint(ref IntPtr node, float mean, float stDev)
        {
            CheckErr(rgl_node_gaussian_noise_angular_hitpoint(ref node, mean, stDev, RGLAxis.RGL_AXIS_Y));
        }

        public static void NodeGaussianNoiseDistance(ref IntPtr node, float mean, float stDev, float stDevRisePerMeter)
        {
            CheckErr(rgl_node_gaussian_noise_distance(ref node, mean, stDev, stDevRisePerMeter));
        }

        public static void NodePointsRemoveGround(ref IntPtr node, float groundAngleThreshold, float groundDistanceThreshold, float groundFilterDistance)
        {
            CheckErr(rgl_node_points_remove_ground(ref node, RGLAxis.RGL_AXIS_Y, groundAngleThreshold, groundDistanceThreshold, groundFilterDistance));
        }

        public static void NodePointsRadarPostprocess(ref IntPtr node, float distanceSeparation, float azimuthSeparation)
        {
            CheckErr(rgl_node_points_radar_postprocess(ref node, distanceSeparation, azimuthSeparation));
        }

        public static void GraphRun(IntPtr node)
        {
            CheckErr(rgl_graph_run(node));
        }

        public static int GraphGetResultSize(IntPtr node, RGLField field)
        {
            Int32 pointCount = 0;
            Int32 pointSize = 0;
            CheckErr(rgl_graph_get_result_size(node, field, out pointCount, out pointSize));
            return (int) pointCount;
        }

        public static int GraphGetResult<T>(IntPtr node, RGLField field, ref T[] data, int expectedPointSize) where T : unmanaged
        {
            Int32 pointCount = 0;
            Int32 pointSize = 0;
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

        public static void GraphSavePcdFile(IntPtr node, string path)
        {
            CheckErr(rgl_graph_write_pcd_file(node, path));
        }

        public static void GraphNodeAddChild(IntPtr parent, IntPtr child)
        {
            CheckErr(rgl_graph_node_add_child(parent, child));
        }

        public static void GraphNodeRemoveChild(IntPtr parent, IntPtr child)
        {
            CheckErr(rgl_graph_node_remove_child(parent, child));
        }

        public static void GraphNodeSetPriority(IntPtr node, Int32 priority)
        {
            CheckErr(rgl_graph_node_set_priority(node, priority));
        }

        public static int GraphNodeGetPriority(IntPtr node)
        {
            CheckErr(rgl_graph_node_get_priority(node, out var outPriority));
            return outPriority;
        }

        public static void GraphDestroy(IntPtr node)
        {
            CheckErr(rgl_graph_destroy(node));
        }
    }
}
