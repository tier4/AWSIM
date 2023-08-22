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
public class Differential : MonoBehaviour 
{
    public CarController carController;
    public WheelController[] wheels;
    public TyreParameters axleParams;
    public bool isFront, isRear;
    public void FixedUpdate()
    {
        if (isRear)
        {   
            
            if (carController.vehicleParams.rearSolidAxle)
            {
                float deltaTorqueDiff = 5f * (wheels[1].omega - wheels[0].omega);
                float deltaTorqueSolid = (wheels[1].Fx - wheels[0].Fx)*axleParams.tyreRadius; // = T2-T1
                wheels[1].driveTorque = (carController.TAxle + deltaTorqueSolid)/2f - deltaTorqueDiff;
                wheels[0].driveTorque = carController.TAxle - wheels[1].driveTorque + deltaTorqueDiff;              
            }
            else
            {
                float deltaTorqueDiff = carController.vehicleParams.rearDifferentialDamping * (wheels[1].omega - wheels[0].omega);
                wheels[1].driveTorque = carController.TAxle/2f - deltaTorqueDiff;
                wheels[0].driveTorque = carController.TAxle/2f + deltaTorqueDiff;
            }

            float rpmShaft = 0.5f*(wheels[1].omega + wheels[0].omega) * 60f/(2*Mathf.PI) 
                            * carController.vehicleParams.gearRatio[carController.gear-1]; // at the engine side of transmission
            carController.rpmEngine = Mathf.Clamp(rpmShaft,960f,carController.vehicleParams.maxEngineRpm) + Random.Range(-10f,10f);
        }
        if (isFront)
        {
            wheels[1].driveTorque = 0f;
            wheels[0].driveTorque = 0f;
        }
    }
}