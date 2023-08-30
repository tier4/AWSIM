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
public class PowertrainDataPublisher : Publisher<PowertrainData>
{
    public string modifiedRosNamespace = "";
    public string modifiedTopicName = "/powertrain_data";
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
    public PowertrainDataSimulator ptSim;
    public override void fillMsg()
    {
        msg.Map_sensor = ptSim.map_sensor;
        msg.Lambda_sensor =  ptSim.lambda_sensor;
        msg.Fuel_level =  ptSim.fuel_level;
        msg.Fuel_pressure = ptSim.fuel_pressure;
        msg.Engine_oil_pressure = ptSim.engine_oil_pressure;
        msg.Engine_oil_temperature = ptSim.engine_oil_temperature;
        msg.Engine_coolant_temperature = ptSim.engine_coolant_temperature;
        msg.Engine_coolant_pressure = ptSim.engine_coolant_pressure;
        msg.Engine_rpm = ptSim.engine_rpm;
        msg.Engine_on_status = ptSim.engine_on_status;
        msg.Engine_run_switch_status = ptSim.engine_run_switch_status;
        msg.Throttle_position = ptSim.throttle_position;
        msg.Current_gear = ptSim.current_gear;
        msg.Gear_shift_status =ptSim.gear_shift_status;
        msg.Transmission_oil_pressure = ptSim.transmission_oil_pressure;
        msg.Transmission_accumulator_pressure =ptSim.transmission_accumulator_pressure;
        msg.Transmission_oil_temperature = ptSim.transmission_oil_temperature;
        msg.Vehicle_speed_kmph = ptSim.vehicle_speed_kmph;
        msg.Torque_wheels_nm = ptSim.torque_wheels_nm;


    }
} // end of class
} // end of autonoma namespace