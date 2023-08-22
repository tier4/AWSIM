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
using System;
[Serializable]
public class VehicleDynamicsData
{
    public float time;
    public VehicleState state;
    public Control control;
    public Tyres tyres;
    public Powertrain powertrain;
    public VehicleDynamicsData()
    {
        time = new float();
        state = new VehicleState();
        control = new Control();
        tyres = new Tyres();
        powertrain = new Powertrain();
        // gps
        // imu
    }

    public void GetValues(CarController carController, WheelController[] scrWheels)
    {
        time += Time.deltaTime;
        // Get values from the CarController object
        state = carController.vehicleState;

        // get values from control
        control.throttleDes = carController.throttleCmd;
        control.throttleFbk = carController.thrApplied;

        control.steerDes = carController.steerAngleCmd;
        control.steerFbk = carController.steerAngleApplied;

        control.brakeDes = carController.brakeCmd;
        control.brakeFbk = carController.brakeApplied;
        control.gear = carController.gear;

        // Get values from the ScrWheel objects
        for (int i = 0; i < scrWheels.Length; i++)
        {
            tyres.sxDir[i] = scrWheels[i].sxDir;
            tyres.syDir[i] = scrWheels[i].syDir;
            tyres.sx[i] = scrWheels[i].sx;
            tyres.sy[i] = -scrWheels[i].sy;
            tyres.Sx[i] = scrWheels[i].S;
            tyres.Sy[i] = scrWheels[i].S;
            tyres.Fx[i] = scrWheels[i].Fx;
            tyres.Fy[i] = -scrWheels[i].Fy;
            tyres.Fz[i] = scrWheels[i].Fz;
            tyres.springDeflection[i] = scrWheels[i].springDeflection;
            tyres.springVelocity[i] = scrWheels[i].springVelocity;
            state.omega[i] = scrWheels[i].omega;
            tyres.omega[i] = scrWheels[i].omega;
        }

        // Get values from Engine object
        powertrain.rpmEngine = carController.powertrain.rpmEngine;
        powertrain.TAxle = carController.powertrain.TAxle;
        powertrain.TEngine = carController.powertrain.TEngine;

        
    }
}

[Serializable]
public class VehicleState
{
    public float[] pos = new float[3];
    public float[] V = new float[3];
    public float rollRate;
    public float pitchRate;
    public float yawRate;
    public float roll;
    public float pitch;
    public float yaw;
    public float[] omega = new float[4];
    

}

[Serializable]
public class Control
{
    public float throttleDes;
    public float steerDes;
    public float brakeDes;
    public float throttleFbk;
    public float steerFbk;
    public float brakeFbk;
    public float clutch;
    public float gear;
}

[Serializable]
public class Tyres
{
    public float[] sxDir = new float[4];
    public float[] syDir = new float[4];
    public float[] sx = new float[4];
    public float[] sy = new float[4];
    public float[] Sx = new float[4];
    public float[] Sy = new float[4];
    public float[] Fx = new float[4];
    public float[] Fy = new float[4];
    public float[] Fz = new float[4];
    public float[] omega = new float[4];
    public float[] springDeflection = new float[4];
    public float[] springVelocity = new float[4];
}

[Serializable]
public class Powertrain
{
    public float TEngine;
    public float TAxle;
    public float rpmEngine;
}