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

namespace RGLUnityPlugin
{
public enum RGLNodeType
	{
		UNKNOWN = -1,
		ANY = 0,
		RAYS_FROM_MAT3X4F = 1,
		RAYS_SET_RANGE,
		RAYS_SET_RING_IDS,
		RAYS_SET_TIME_OFFSETS,
		RAYS_TRANSFORM,
		POINTS_TRANSFORM,
		POINTS_FORMAT,
		POINTS_YIELD,
		POINTS_COMPACT,
		POINTS_DOWNSAMPLE,
		POINTS_TEMPORAL_MERGE,
		POINTS_ROS2_PUBLISH,
		POINTS_UDP_PUBLISH,
		RAYTRACE,
		GAUSSIAN_NOISE_ANGULAR_RAY,
		GAUSSIAN_NOISE_ANGULAR_HITPOINT,
		GAUSSIAN_NOISE_DISTANCE,
	};
}