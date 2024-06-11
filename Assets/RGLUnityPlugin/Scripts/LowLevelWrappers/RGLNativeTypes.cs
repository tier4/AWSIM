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

namespace RGLUnityPlugin
{
	public enum RGLStatus : Int32
	{
		SUCCESS = 0,
		INVALID_ARGUMENT,
		INVALID_STATE,
		LOGGING_ERROR,
		INVALID_API_OBJECT,
		INVALID_FILE_PATH,
		TAPE_ERROR,
		UDP_ERROR,
		ROS2_ERROR,
		INVALID_PIPELINE,
		INITIALIZATION_ERROR,
		NOT_IMPLEMENTED = 404,
		INTERNAL_EXCEPTION = 500,
	};

	public enum RGLField : Int32
	{
		XYZ_VEC3_F32 = 1,
		INTENSITY_F32,
		IS_HIT_I32,
		IS_GROUND_I32,
		RAY_IDX_U32,
		ENTITY_ID_I32,
		DISTANCE_F32,
		AZIMUTH_F32,
		ELEVATION_F32,
		RING_ID_U16,
		RETURN_TYPE_U8,
		TIME_STAMP_F64,
		ABSOLUTE_VELOCITY_VEC3_F32,
		RELATIVE_VELOCITY_VEC3_F32,
		RADIAL_SPEED_F32,
		POWER_F32,
		RCS_F32,
		NOISE_F32,
		SNR_F32,
		NORMAL_VEC3_F32,
		INCIDENT_ANGLE_F32,
		RAY_POSE_MAT3x4_F32,
		// Dummy fields
		PADDING_8 = 1024,
		PADDING_16,
		PADDING_32,
		// Dynamic fields
		DYNAMIC_FORMAT = 13842,
	}

	public enum RGLLogLevel : Int32
	{
		ALL = 0,
		TRACE = 0,
		DEBUG = 1,
		INFO = 2,
		WARN = 3,
		ERROR = 4,
		CRITICAL = 5,
		OFF = 6,
	};

	public enum RGLAxis : Int32
	{
		RGL_AXIS_X = 1,
		RGL_AXIS_Y = 2,
		RGL_AXIS_Z = 3,
	};

	public enum RGLLidarModel : Int32
	{
		RGL_VELODYNE_VLP16 = 1,
		RGL_VELODYNE_VLP32C = 2,
		RGL_VELODYNE_VLS128 = 3,
		RGL_HESAI_PANDAR_40P = 4,
		RGL_HESAI_PANDAR_QT64 = 5,
		RGL_HESAI_QT128C2X = 6,
		RGL_HESAI_PANDAR_128E4X = 7,
		RGL_HESAI_PANDAR_XT32 = 8,
	};

	// Items have been renamed to be displayed in Unity nicer.
	public enum RGLReturnMode : UInt32
	{
		/* RGL_RETURN_FIRST */ SingleReturnFirst = 1 << 29, // Three of the most significant bits encode the number of returns
		/* RGL_RETURN_SECOND */ SingleReturnSecond,
		/* RGL_RETURN_LAST */ SingleReturnLast,
		/* RGL_RETURN_STRONGEST */ SingleReturnStrongest,
		/* RGL_RETURN_LAST_STRONGEST */ DualReturnLastStrongest = 2 << 29,
		/* RGL_RETURN_FIRST_LAST */ DualReturnFirstLast,
		/* RGL_RETURN_FIRST_STRONGEST */ DualReturnFirstStrongest,
		/* RGL_RETURN_STRONGEST_SECOND_STRONGEST */ DualReturnStrongestSecondStrongest,
		/* RGL_RETURN_FIRST_SECOND */ DualReturnFirstSecond,
	};

	public enum RGLUdpOptions : UInt32
	{
		RGL_UDP_NO_ADDITIONAL_OPTIONS       = 0,
		RGL_UDP_ENABLE_HESAI_UDP_SEQUENCE   = 1 << 0,
		RGL_UDP_HIGH_RESOLUTION_MODE        = 1 << 1,
		RGL_UDP_UP_CLOSE_BLOCKAGE_DETECTION = 1 << 2,
	};

	public enum RGLQosPolicyReliability
	{
		QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0,
		QOS_POLICY_RELIABILITY_RELIABLE = 1,
		QOS_POLICY_RELIABILITY_BEST_EFFORT = 2,
	};

	public enum RGLQosPolicyDurability
	{
		QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0,
		QOS_POLICY_DURABILITY_TRANSIENT_LOCAL = 1,
		QOS_POLICY_DURABILITY_VOLATILE = 2,
	};

	public enum RGLQosPolicyHistory
	{
		QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0,
		QOS_POLICY_HISTORY_KEEP_LAST = 1,
		QOS_POLICY_HISTORY_KEEP_ALL = 2,
	};

	public enum RGLExtension : Int32
	{
		RGL_EXTENSION_PCL = 0,
		RGL_EXTENSION_ROS2 = 1,
		RGL_EXTENSION_UDP = 2,
		RGL_EXTENSION_SNOW = 3,
		RGL_EXTENSION_COUNT
	};

	public enum RGLReturnType : Int32
	{
		RGL_RETURN_TYPE_NOT_DIVERGENT = 0,
		RGL_RETURN_TYPE_FIRST = 1,
		RGL_RETURN_TYPE_LAST = 2,
	}
}
