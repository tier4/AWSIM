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
public class HeadMovement : MonoBehaviour
{
    public float input,prevInput;
    public float diff;
    public float ax,axPrev,ay,ayPrev,az,azPrev;

    public float freq1,freq2,gainX,gainY,gainZ,height;

    public float gainRotXaX,gainRotXaZ,gainRotXrandom;
    public float gainRotYaY,gainRotZay;
    public float gainRotYrandom,gainRotZrandom;
    public ImuSimulator imuSimulator;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        bool isMoving = imuSimulator.imuVelLocal.x > 1f;
        ax = HelperFunctions.lowPassFirstOrder(imuSimulator.imuAccel.x, axPrev,30f);
        ax = Mathf.Clamp(ax,-15f,15f);
        ay = HelperFunctions.lowPassFirstOrder(imuSimulator.imuAccel.y, ayPrev,30f);
        ay = Mathf.Clamp(ay,-15f,15f);
        az = HelperFunctions.lowPassFirstOrder(imuSimulator.imuAccel.z, azPrev,30f);
        az = Mathf.Clamp(az,-15f,15f);
        axPrev=ax;
        ayPrev=ay;
        azPrev=az;
        if (!isMoving)
        {
            ax = 0f;
            ay = 0f;
            az = 9.81f;
        }
        transform.localPosition = new Vector3(ay*gainY, az*gainZ + height, 0.34f+ax*gainX);

        transform.localRotation = Quaternion.Euler(
            ax*gainRotXaX + (az - 9.81f) * gainRotXaZ + Random.Range(-1f,1f) * gainRotXrandom * imuSimulator.imuVelLocal.x,
        ay*gainRotYaY + Random.Range(-1f,1f) * gainRotYrandom * imuSimulator.imuVelLocal.x,
        ay*gainRotZay + Random.Range(-1f,1f) * gainRotZrandom * imuSimulator.imuVelLocal.x);



    }
}
}