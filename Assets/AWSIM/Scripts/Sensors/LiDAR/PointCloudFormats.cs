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

using System.Collections.Generic;
using RGLUnityPlugin;

/// <summary>
/// Each of the following classes describes the point format (fields) for ROS2 and RGL.
/// </summary>
namespace AWSIM.PointCloudFormats
{
    /// <summary>
    /// 24-byte format used by Autoware
    /// </summary>
    public static class FormatPCL24
    { 
        public static RGLField[] GetRGLFields()
        {
            return new[]
            {
                RGLField.XYZ_VEC3_F32,
                RGLField.PADDING_32,
                RGLField.INTENSITY_F32,
                RGLField.RING_ID_U16,
                RGLField.PADDING_16
            };
        }
    }

    /// <summary>
    /// 48-byte format used by Autoware
    /// </summary>
    public static class FormatPCL48
    {
        public static RGLField[] GetRGLFields()
        {
            return new[]
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
            };
        }
    }

    /// <summary>
    /// Machine learning format for instance/semantic segmentation tasks
    /// </summary>
    public static class FormatMLInstanceSegmentation
    {
        public static RGLField[] GetRGLFields()
        {
            return new[]
            {
                RGLField.XYZ_VEC3_F32,
                RGLField.ENTITY_ID_I32,
                RGLField.INTENSITY_F32
            };
        }
    }
}