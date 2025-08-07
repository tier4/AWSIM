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

namespace Awsim.Entity
{
    /// <summary>
    /// Vehicle ROS2 utility static class.
    /// </summary>
    public static class V2IRos2Converter
    {
        /// <summary>
        /// Convert the TrafficLight.BulbType to ROS msg TrafficLightElement
        /// </summary>
        /// <param name="type"></param>
        /// <returns>Converted BulbType.</returns>
        /// 
        public static byte UnityToRosBulbShape(TrafficLight.BulbType type)
        {
            if (type == TrafficLight.BulbType.AnyCircleBulb || type == TrafficLight.BulbType.RedBulb || type == TrafficLight.BulbType.YellowBulb || type == TrafficLight.BulbType.GreenBulb)
                return autoware_perception_msgs.msg.TrafficLightElement.CIRCLE;
            else if (type == TrafficLight.BulbType.LeftArrowBulb)
                return autoware_perception_msgs.msg.TrafficLightElement.LEFT_ARROW;
            else if (type == TrafficLight.BulbType.RightArrowBulb)
                return autoware_perception_msgs.msg.TrafficLightElement.RIGHT_ARROW;
            else if (type == TrafficLight.BulbType.UpArrowBulb)
                return autoware_perception_msgs.msg.TrafficLightElement.UP_ARROW;
            else if (type == TrafficLight.BulbType.DownArrowBulb)
                return autoware_perception_msgs.msg.TrafficLightElement.DOWN_ARROW;
            else if (type == TrafficLight.BulbType.DownLeftArrowBulb)
                return autoware_perception_msgs.msg.TrafficLightElement.DOWN_LEFT_ARROW;
            else if (type == TrafficLight.BulbType.DownRightArrowBulb)
                return autoware_perception_msgs.msg.TrafficLightElement.DOWN_RIGHT_ARROW;
            else if (type == TrafficLight.BulbType.CrossBulb)
                return autoware_perception_msgs.msg.TrafficLightElement.CROSS;
            else
                return autoware_perception_msgs.msg.TrafficLightElement.UNKNOWN;
        }

        /// <summary>
        /// Convert the TrafficLight.BulbStatus to ROS msg TrafficLightElement
        /// </summary>
        /// <param name="status"></param>
        /// <returns>Converted BulbStatus.</returns>
        /// 
        public static byte UnityToRosBulbStatus(TrafficLight.BulbStatus status)
        {
            if (status == TrafficLight.BulbStatus.SolidOff)
                return autoware_perception_msgs.msg.TrafficLightElement.SOLID_OFF;
            else if (status == TrafficLight.BulbStatus.SolidOn)
                return autoware_perception_msgs.msg.TrafficLightElement.SOLID_ON;
            else if (status == TrafficLight.BulbStatus.Frashing)
                return autoware_perception_msgs.msg.TrafficLightElement.FLASHING;
            else
                return autoware_perception_msgs.msg.TrafficLightElement.UNKNOWN;
        }

        /// <summary>
        /// Convert the TrafficLight.BulbColor to ROS msg TrafficLightElement
        /// </summary>
        /// <param name="color"></param>
        /// <returns>Converted BulbColor.</returns>
        /// 
        public static byte UnityToRosBulbColor(TrafficLight.BulbColor color)
        {
            if (color == TrafficLight.BulbColor.Red)
                return autoware_perception_msgs.msg.TrafficLightElement.RED;
            else if (color == TrafficLight.BulbColor.Yellow)
                return autoware_perception_msgs.msg.TrafficLightElement.AMBER;
            else if (color == TrafficLight.BulbColor.Green)
                return autoware_perception_msgs.msg.TrafficLightElement.GREEN;
            else if (color == TrafficLight.BulbColor.White)
                return autoware_perception_msgs.msg.TrafficLightElement.WHITE;
            else
                return autoware_perception_msgs.msg.TrafficLightElement.UNKNOWN;
        }

    }
}