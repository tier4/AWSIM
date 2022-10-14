// Copyright 2019-2021 Robotec.ai.
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

using UnityEngine;

namespace ROS2
{
/// <summary>
/// A set of transformation functions between coordinate systems of Unity and ROS
/// </summary>
public static class Transformations
{
    public static Vector3 Ros2Unity(this Vector3 vector3)
    {
        return new Vector3(-vector3.y, vector3.z, vector3.x);
    }

    public static Vector3 Unity2Ros(this Vector3 vector3)
    {
        return new Vector3(vector3.z, -vector3.x, vector3.y);
    }

    public static Vector3 Ros2UnityScale(this Vector3 vector3)
    {
        return new Vector3(vector3.y, vector3.z, vector3.x);
    }

    public static Vector3 Unity2RosScale(this Vector3 vector3)
    {
        return new Vector3(vector3.z, vector3.x, vector3.y);
    }

    public static Quaternion Ros2Unity(this Quaternion quaternion)
    {
        return new Quaternion(quaternion.y, -quaternion.z, -quaternion.x, quaternion.w);
    }

    public static Quaternion Unity2Ros(this Quaternion quaternion)
    {
        return new Quaternion(-quaternion.z, quaternion.x, -quaternion.y, quaternion.w);
    }

    public static void Unity2Ros(ref Quaternion quaternion)
    {
        var z = quaternion.z;
        var x = quaternion.x;
        var y = quaternion.y;
        quaternion.x = -z;
        quaternion.y = x;
        quaternion.z = -y;
    }

    public static void Unity2Ros(ref Vector3 vector)
    {
        var z = vector.z;
        var x = vector.x;
        var y = vector.y;
        vector.x = z;
        vector.y = -x;
        vector.z = y;
    }

    public static Matrix4x4 Unity2RosMatrix4x4()
    {
        // Note: The matrix here is written as-if on paper,
        // but Unity's Matrix4x4 is constructed from column-vectors, hence the transpose.
        return new Matrix4x4(
            new Vector4( 0.0f, 0.0f, 1.0f, 0.0f),
            new Vector4(-1.0f, 0.0f, 0.0f, 0.0f),
            new Vector4( 0.0f, 1.0f, 0.0f, 0.0f),
            new Vector4( 0.0f, 0.0f, 0.0f, 1.0f)
        ).transpose;
    }
}

}  // namespace ROS2
