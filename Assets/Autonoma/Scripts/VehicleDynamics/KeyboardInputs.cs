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
using VehicleDynamics;

public class KeyboardInputs : MonoBehaviour
{
    public CarController carController;
    public float throttle,brake,steering;
    public bool gearUp,gearDown;
    void Start()
    {
        carController = HelperFunctions.GetParentComponent<CarController>(transform);
    }
    void Update()
    {
        float throttleInput = Mathf.Clamp(Input.GetAxisRaw("Throttle"),0f,1f);
        float brakeInput = Mathf.Clamp(Input.GetAxisRaw("Brake"),0f,1f);
        if (throttleInput > throttle)
            throttle = HelperFunctions.rateLimitUpdate(throttleInput,throttle,1.5f);
        else
            throttle = HelperFunctions.rateLimitUpdate(throttleInput,throttle,5f);
        if (brakeInput > brake)
            brake = HelperFunctions.rateLimitUpdate(brakeInput,brake,3.5f);
        else
            brake = HelperFunctions.rateLimitUpdate(brakeInput,brake,5f);
        if (Mathf.Abs(Input.GetAxisRaw("Steering")) > Mathf.Abs(steering))
            steering = HelperFunctions.rateLimitUpdate(Input.GetAxisRaw("Steering"),steering,3.5f);
        else
            steering = HelperFunctions.rateLimitUpdate(Input.GetAxisRaw("Steering"),steering,5f);

        gearUp = Input.GetKey(KeyCode.Tab);
        gearDown = Input.GetKey(KeyCode.CapsLock) || Input.GetKey(KeyCode.LeftShift);

        carController.physicalActuator = true;
        carController.gearUp = gearUp;
        carController.gearDown = gearDown;
        carController.steerAngleCmd = carController.vehicleParams.maxSteeringAngle*-steering/carController.vehicleParams.steeringRatio;
        carController.throttleCmd = throttle;
        carController.brakeCmd = brake*carController.vehicleParams.maxBrakeKpa;

        carController.throttleCmd = Mathf.Clamp(carController.throttleCmd,0f,1f);
        carController.brakeCmd = Mathf.Clamp(carController.brakeCmd,0f,carController.vehicleParams.maxBrakeKpa);
    }
}