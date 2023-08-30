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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using VehicleDynamics;

namespace Autonoma
{
public class VehicleDataSimulator : MonoBehaviour
{
    public CarController carController;
    public WheelController[] wheelControllers;
    public RaptorSM sm;
    public RaceControl raceControl;
    public float fl_tire_temperature;
    public float fl_damper_linear_potentiometer;
    public float fl_tire_pressure;
    public float fl_tire_pressure_gauge;
    public float fl_wheel_load;
    public float fr_tire_temperature;
    public float fr_damper_linear_potentiometer;
    public float fr_tire_pressure;
    public float fr_tire_pressure_gauge;
    public float fr_wheel_load;
    public float rl_tire_temperature;
    public float rl_damper_linear_potentiometer;
    public float rl_tire_pressure;
    public float rl_tire_pressure_gauge;
    public float rl_wheel_load;
    public float rr_tire_temperature;
    public float rr_damper_linear_potentiometer;
    public float rr_tire_pressure;
    public float rr_tire_pressure_gauge;
    public float rr_wheel_load;
    //  Brake temps
    public float fl_brake_temp;
    public float fr_brake_temp;
    public float rl_brake_temp;
    public float rr_brake_temp;
    //  Misc data
    public float battery_voltage;
    public byte safety_switch_state;
    public bool mode_switch_state;
    public byte sys_state;
    public byte ct_state;
    //  Accel pedal report 
    public float accel_pedal_input;
    //  0 to 100%
    public float accel_pedal_output;
    //  0 to 100%
    //  Brake report
    public float front_brake_pressure;
    //  kPa
    public float rear_brake_pressure;
    //  kPa
    //  Steering Report
    public float steering_wheel_angle;
    //  degrees
    public float steering_wheel_angle_cmd;
    //  degrees
    public float steering_wheel_torque;
    //  0 to 100%
    //  Wheel speeds (kph)
    public float ws_front_left;
    public float ws_front_right;
    public float ws_rear_left;
    public float ws_rear_right;
    void Start()
    {
        carController = HelperFunctions.GetParentComponent<CarController>(transform);
        Transform mainTransform = HelperFunctions.GetParentTransform(transform);
        wheelControllers = mainTransform.GetComponentsInChildren<WheelController>();     
    }
    void FixedUpdate()
    {
        sys_state = (byte)sm.current_sys;
        ct_state = (byte)sm.current_ct;

        fl_tire_temperature = wheelControllers[0].currTyreTemp;
        fl_damper_linear_potentiometer = 0.0f;
        fl_tire_pressure = 35.0f;
        fl_tire_pressure_gauge = 0.0f;
        fl_wheel_load = wheelControllers[0].Fz;

        fr_tire_temperature = wheelControllers[1].currTyreTemp;
        fr_damper_linear_potentiometer = 0.0f;
        fr_tire_pressure = 35.0f;
        fr_tire_pressure_gauge = 0.0f;
        fr_wheel_load = wheelControllers[1].Fz;

        rl_tire_temperature = wheelControllers[2].currTyreTemp;
        rl_damper_linear_potentiometer = 0.0f;
        rl_tire_pressure = 35.0f;
        rl_tire_pressure_gauge = 0.0f;
        rl_wheel_load = wheelControllers[2].Fz;

        rr_tire_temperature = wheelControllers[3].currTyreTemp;
        rr_damper_linear_potentiometer = 0.0f;
        rr_tire_pressure = 35.0f;
        rr_tire_pressure_gauge = 0.0f;
        rr_wheel_load = wheelControllers[3].Fz;

        fl_brake_temp = 50.0f;
        fr_brake_temp = 50.0f;
        rl_brake_temp = 50.0f;
        rr_brake_temp = 50.0f;

        battery_voltage = 13.0f;
        safety_switch_state = 0;
        mode_switch_state = true;
        accel_pedal_input = carController.throttleCmd * 100;
        accel_pedal_output = carController.thrApplied *100;
        front_brake_pressure = carController.brakeApplied *   carController.vehicleParams.brakeBias;  // TODO: check this
        rear_brake_pressure = carController.brakeApplied * (1-  carController.vehicleParams.brakeBias);
        steering_wheel_angle = carController.steerAngleApplied*  carController.vehicleParams.steeringRatio;
        steering_wheel_angle_cmd = carController.steerAngleCmd*  carController.vehicleParams.steeringRatio;
        steering_wheel_torque = 0.0f;

        ws_front_left =  wheelControllers[0].omega > 1.5f ?  wheelControllers[0].omega *   carController.vehicleParams.frontTyreParams.tyreRadius * 3.6f : 0.0f;
        ws_front_right =  wheelControllers[1].omega > 1.5f ? wheelControllers[1].omega  *   carController.vehicleParams.frontTyreParams.tyreRadius * 3.6f : 0.0f;
        ws_rear_left =  wheelControllers[2].omega  > 1.5f ? wheelControllers[2].omega  *   carController.vehicleParams.rearTyreParams.tyreRadius * 3.6f : 0.0f;
        ws_rear_right =  wheelControllers[3].omega > 1.5f ? wheelControllers[3].omega  *   carController.vehicleParams.rearTyreParams.tyreRadius * 3.6f : 0.0f;
    }
}
}