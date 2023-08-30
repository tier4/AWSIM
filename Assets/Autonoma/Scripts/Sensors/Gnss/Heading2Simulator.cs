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

public class Heading2Simulator : MonoBehaviour
{
    public float heading2;
    public Vector3 secondaryAntPos;
    void Start() 
    {
        secondaryAntPos = transform.localPosition;   
    } 
    void FixedUpdate()
    {   
        Vector3 imuAngle = transform.eulerAngles;
        for(int i = 0; i<3; i++)
        {
            imuAngle[i] = imuAngle[i] > 180f ? imuAngle[i] - 360f : imuAngle[i];
        }
        imuAngle = HelperFunctions.unity2vehDynCoord(-imuAngle);
        float relativeAngle = Mathf.Atan2(-secondaryAntPos.z,secondaryAntPos.x)*180f/Mathf.PI;
        // CW +, [deg], NORTH = 90 (-EAST = 0). 0-360
        heading2 = HelperFunctions.MathMod((180f - imuAngle.z + relativeAngle ),360f);

    }
}

