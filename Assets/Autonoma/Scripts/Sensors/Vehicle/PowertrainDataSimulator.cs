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
public class PowertrainDataSimulator : MonoBehaviour
{
    public CarController carController;
    public WheelController[] wheelControllers;
    public float map_sensor;
    public float lambda_sensor;
    public float fuel_level;
    public float fuel_pressure;
    public float engine_oil_pressure;
    public float engine_oil_temperature;
    public float engine_coolant_temperature;
    public float engine_coolant_pressure;
    public float engine_rpm;
    public bool engine_on_status;
    public bool engine_run_switch_status;
    public float throttle_position;
    public sbyte current_gear;
    public sbyte gear_shift_status;
    public float transmission_oil_pressure;
    public float transmission_accumulator_pressure;
    public float transmission_oil_temperature;
    public float vehicle_speed_kmph;
    public float torque_wheels_nm;
    void Start()
    {
        carController = HelperFunctions.GetParentComponent<CarController>(transform);
        Transform mainTransform = HelperFunctions.GetParentTransform(transform);
        wheelControllers = mainTransform.GetComponentsInChildren<WheelController>();     
    }
    void FixedUpdate()
    {
        map_sensor = 30f;
        lambda_sensor = 0.85f;
        fuel_level = 1f;
        fuel_pressure = 60f;
        engine_oil_pressure = 40.0f;
        engine_oil_temperature = 80.0f;
        engine_coolant_temperature = 80.0f;
        engine_coolant_pressure = 40.0f;
        engine_rpm = carController.rpmEngine;
        engine_on_status = true;
        engine_run_switch_status = false;
        throttle_position = carController.thrApplied* 100f;
        current_gear = (sbyte)carController.gear;
        gear_shift_status = 0;
        transmission_oil_pressure = 40.0f;
        transmission_accumulator_pressure = 40.0f;
        transmission_oil_temperature = 50.0f;
        vehicle_speed_kmph = (carController.V.x*3.6f > 1.5f) ? carController.V.x*3.6f : 0.0f;
        torque_wheels_nm = carController.TAxle;   
    }
}
}