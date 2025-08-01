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
using System.Collections.Generic;
using RGLUnityPlugin;

namespace Awsim.Entity
{
    public enum PointCloudFormat : UInt32
    {
        Pcl24,
        Pcl48,
        PointXYZIRCAEDT,
        MLInstanceSegmentation,
        RadarSmartMicro,
        Custom,
    }

    public static class PointCloudFormatLibrary
    {
        /// <summary>
        /// Each of the following entries describes the point format (fields) for ROS2 and RGL.
        /// </summary>
        public static readonly Dictionary<PointCloudFormat, RGLField[]> ByFormat =
            new Dictionary<PointCloudFormat, RGLField[]>
            {
                // 24-byte format used by Autoware
                {PointCloudFormat.Pcl24, new[]
                {
                    RGLField.XYZ_VEC3_F32,
                    RGLField.PADDING_32,
                    RGLField.INTENSITY_F32,
                    RGLField.RING_ID_U16,
                    RGLField.PADDING_16
                }},
                // 48-byte format used by Autoware
                {PointCloudFormat.Pcl48, new[]
                {
                    RGLField.XYZ_VEC3_F32,
                    RGLField.PADDING_32,
                    RGLField.INTENSITY_F32,
                    RGLField.RING_ID_U16,
                    RGLField.PADDING_16,
                    RGLField.AZIMUTH_F32,
                    RGLField.DISTANCE_F32,
                    RGLField.RETURN_TYPE_U8,
                    RGLField.PADDING_8,
                    RGLField.PADDING_16,
                    RGLField.PADDING_32,
                    RGLField.TIME_STAMP_F64
                }},
                // PointXYZIRCAEDT format used by Autoware
                {PointCloudFormat.PointXYZIRCAEDT, new[]
                {
                    RGLField.XYZ_VEC3_F32,
                    RGLField.INTENSITY_U8,
                    RGLField.RETURN_TYPE_U8,
                    RGLField.RING_ID_U16,
                    RGLField.AZIMUTH_F32,
                    RGLField.ELEVATION_F32,
                    RGLField.DISTANCE_F32,
                    RGLField.TIME_STAMP_U32
                }},
                // Machine learning format for instance/semantic segmentation tasks
                {PointCloudFormat.MLInstanceSegmentation, new[]
                {
                    RGLField.XYZ_VEC3_F32,
                    RGLField.ENTITY_ID_I32,
                    RGLField.INTENSITY_F32
                }},
                // Format used in Radar Smart Micro
                {PointCloudFormat.RadarSmartMicro, new[]
                {
                    RGLField.XYZ_VEC3_F32,
                    RGLField.RADIAL_SPEED_F32,
                    RGLField.POWER_F32,
                    RGLField.RCS_F32,
                    RGLField.NOISE_F32,
                    RGLField.SNR_F32
                }},
                // Custom format (empty by default)
                {PointCloudFormat.Custom, Array.Empty<RGLField>()},
            };
    }
}