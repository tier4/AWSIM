using NUnit.Framework;
using AWSIM;

using autoware_auto_vehicle_msgs.msg;

namespace Ego
{
    public class TurnSignalInput
    {
        [OneTimeSetUp]
        public void OneTimeSetUp() {
            // ROS 2 Unity handler is required to load ROS 2 message assemblies
            var ros2UnityCore = new ROS2.ROS2UnityCore();
        }

        public static object[] ros2UnityHazardTestCases =
        {
            new object[] { autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.DISABLE, Vehicle.TurnSignal.NONE },
            new object[] { autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_LEFT, Vehicle.TurnSignal.LEFT },
            new object[] { autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_RIGHT, Vehicle.TurnSignal.RIGHT },
            new object[] { autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.NO_COMMAND, Vehicle.TurnSignal.NONE },
        };
        [TestCaseSource(nameof(ros2UnityHazardTestCases))]
        public void Ros2Unity(byte light, Vehicle.TurnSignal result)
        {
            var command = new TurnIndicatorsCommand() {Command = light};
            Assert.AreEqual(result, VehicleROS2Utility.RosToUnityTurnSignal(command));
        }

        public static object[] ros2UnityTurnSignalTestCases =
        {
            new object[] { Vehicle.TurnSignal.NONE, autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.DISABLE},
            new object[] { Vehicle.TurnSignal.LEFT, autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_LEFT},
            new object[] { Vehicle.TurnSignal.RIGHT, autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_RIGHT},
            new object[] { Vehicle.TurnSignal.NONE, autoware_auto_vehicle_msgs.msg.TurnIndicatorsCommand.DISABLE},
        };
        [TestCaseSource(nameof(ros2UnityTurnSignalTestCases))]
        public void Unity2Ros(Vehicle.TurnSignal signal, byte result)
        {
            Assert.AreEqual(VehicleROS2Utility.UnityToRosTurnSignal(signal), result);
        }
    }

    public class GearsInput
    {
        [OneTimeSetUp]
        public void OneTimeSetUp() {
            // ROS 2 Unity handler is required to load ROS 2 message assemblies
            var ros2UnityCore = new ROS2.ROS2UnityCore();
            ros2UnityCore.DestroyNow();
        }

        public static object[] ros2UnityGearTestCases =
        {
            new object[] { autoware_auto_vehicle_msgs.msg.GearReport.NONE, Vehicle.Shift.PARKING },
            new object[] { autoware_auto_vehicle_msgs.msg.GearReport.PARK, Vehicle.Shift.PARKING },
            new object[] { autoware_auto_vehicle_msgs.msg.GearReport.NEUTRAL, Vehicle.Shift.NEUTRAL },
            new object[] { autoware_auto_vehicle_msgs.msg.GearReport.REVERSE, Vehicle.Shift.REVERSE },
            new object[] { autoware_auto_vehicle_msgs.msg.GearReport.DRIVE, Vehicle.Shift.DRIVE },
            new object[] { autoware_auto_vehicle_msgs.msg.GearReport.LOW, Vehicle.Shift.DRIVE },
        };
        [TestCaseSource(nameof(ros2UnityGearTestCases))]
        public void Ros2Unity(byte gear, Vehicle.Shift result)
        {
            var command = new GearCommand() {Command = gear};
            Assert.AreEqual(result, VehicleROS2Utility.RosToUnityShift(command));
        }

        public static object[] unityToRosGearTestCases =
        {
            new object[] { Vehicle.Shift.PARKING, autoware_auto_vehicle_msgs.msg.GearReport.PARK },
            new object[] { Vehicle.Shift.NEUTRAL, autoware_auto_vehicle_msgs.msg.GearReport.NEUTRAL },
            new object[] { Vehicle.Shift.REVERSE, autoware_auto_vehicle_msgs.msg.GearReport.REVERSE },
            new object[] { Vehicle.Shift.DRIVE, autoware_auto_vehicle_msgs.msg.GearReport.DRIVE },
        };
        [TestCaseSource(nameof(unityToRosGearTestCases))]
        public void Unity2Ros(Vehicle.Shift gear, byte result)
        {
            Assert.AreEqual(VehicleROS2Utility.UnityToRosShift(gear), result);
        }
    }
    
}
