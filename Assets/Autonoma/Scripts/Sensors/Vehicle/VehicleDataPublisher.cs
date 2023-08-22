/* 
Copyright 2023 Autonoma, Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:

    http://www.apache.org/licenses/LICENSE-2.0

The software is provided "AS IS", WITHOUT WARRANTY OF ANY KIND, 
express or implied. In no event shall the authors or copyright 
holders be liable for any claim, damages or other liability, 
whether in action of contract, tort or otherwise, arising from, 
out of or in connection with the software or the use of the software.
*/
using UnityEngine;
using autonoma_msgs.msg;

namespace Autonoma
{
public class VehicleDataPublisher : Publisher<VehicleData>
{
    public string modifiedRosNamespace = "";
    public string modifiedTopicName = "/vehicle_data";
    public float modifiedFrequency = 100f;
    public string modifiedFrameId = "";
    public void getPublisherParams()
    {
        // get things from sensor assigned by ui to the sensor
    }
    protected override void Start()
    {
        getPublisherParams();
        this.rosNamespace = modifiedRosNamespace;
        this.topicName = modifiedTopicName;
        this.frequency = modifiedFrequency; // Hz
        this.frameId = modifiedFrameId;
        base.Start();
    }
    public VehicleDataSimulator vehSim;
    public RaptorSM sm;
    public override void fillMsg()
    {
        msg.Fl_tire_temperature = vehSim.fl_tire_temperature;
        msg.Fl_damper_linear_potentiometer = vehSim.fl_damper_linear_potentiometer;
        msg.Fl_tire_pressure = vehSim.fl_tire_pressure;
        msg.Fl_tire_pressure_gauge = vehSim.fl_tire_pressure_gauge;
        msg.Fl_wheel_load = vehSim.fl_wheel_load;
        msg.Fl_brake_temp = vehSim.fl_brake_temp;

        msg.Fr_tire_temperature = vehSim.fr_tire_temperature;
        msg.Fr_damper_linear_potentiometer = vehSim.fr_damper_linear_potentiometer;
        msg.Fr_tire_pressure = vehSim.fr_tire_pressure;
        msg.Fr_tire_pressure_gauge = vehSim.fr_tire_pressure_gauge;
        msg.Fr_wheel_load = vehSim.fr_wheel_load;
        msg.Fr_brake_temp = vehSim.fr_brake_temp;

        msg.Rl_tire_temperature = vehSim.rl_tire_temperature;
        msg.Rl_damper_linear_potentiometer = vehSim.rl_damper_linear_potentiometer;
        msg.Rl_tire_pressure = vehSim.rl_tire_pressure;
        msg.Rl_tire_pressure_gauge = vehSim.rl_tire_pressure_gauge;
        msg.Rl_wheel_load = vehSim.rl_wheel_load;
        msg.Rl_brake_temp = vehSim.rl_brake_temp;

        msg.Rr_tire_temperature = vehSim.rr_tire_temperature;
        msg.Rr_damper_linear_potentiometer = vehSim.rr_damper_linear_potentiometer;
        msg.Rr_tire_pressure = vehSim.rr_tire_pressure;
        msg.Rr_tire_pressure_gauge = vehSim.rr_tire_pressure_gauge;
        msg.Rr_wheel_load = vehSim.rr_wheel_load;
        msg.Rr_brake_temp = vehSim.rr_brake_temp;

        msg.Battery_voltage = vehSim.battery_voltage;
        msg.Safety_switch_state = vehSim.safety_switch_state;
        msg.Mode_switch_state = vehSim.mode_switch_state;
        msg.Accel_pedal_input = vehSim.accel_pedal_input;
        msg.Accel_pedal_output = vehSim.accel_pedal_output;
        msg.Front_brake_pressure = vehSim.front_brake_pressure;
        msg.Rear_brake_pressure = vehSim.rear_brake_pressure;
        msg.Steering_wheel_angle = vehSim.steering_wheel_angle;
        msg.Steering_wheel_angle_cmd = vehSim.steering_wheel_angle_cmd;
        msg.Steering_wheel_torque = vehSim.steering_wheel_torque;
        msg.Sys_state = vehSim.sys_state;

        msg.Ws_front_left = vehSim.ws_front_left;
        msg.Ws_front_right = vehSim.ws_front_right;
        msg.Ws_rear_left = vehSim.ws_rear_left;
        msg.Ws_rear_right = vehSim.ws_rear_right;

    }
} // end of class
} // end of autonoma namespace