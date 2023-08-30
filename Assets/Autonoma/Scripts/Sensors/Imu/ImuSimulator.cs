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
public class ImuSimulator : MonoBehaviour
{

    public Vector3 imuGyro; // [rad/s]
    public Vector3 imuAccel; // [m/s^2]
    public Vector3 imuAngle; // [deg]
    public Vector3 imuVelLocal,imuVelLocalPrev;
    public Rigidbody rb;
    void Start()
    {
        rb = HelperFunctions.GetParentComponent<Rigidbody>(transform);
    }
    void FixedUpdate()
    {   
        
        Vector3 localAngularvelocity = transform.InverseTransformDirection(rb.angularVelocity);
        imuGyro = HelperFunctions.unity2vehDynCoord(-localAngularvelocity);

        imuVelLocal = HelperFunctions.unity2vehDynCoord( transform.InverseTransformDirection( rb.GetPointVelocity( transform.position ) ) );
        Vector3 dvdt = (imuVelLocal - imuVelLocalPrev)/Time.fixedDeltaTime;
        Vector3 localGravity = transform.InverseTransformDirection(Physics.gravity);
        imuAccel = dvdt - Vector3.Cross(imuVelLocal,imuGyro) - HelperFunctions.unity2vehDynCoord(localGravity); //
        imuVelLocalPrev = imuVelLocal;

        // euler angles; some sensors output it with their internal fusion algorithms.
        imuAngle = transform.eulerAngles;
        for(int i = 0; i<3; i++)
        {
            imuAngle[i] = imuAngle[i] > 180f ? imuAngle[i] - 360f : imuAngle[i];
        }
        // RPY, RHS +, [deg], NORTH = 0 for yaw, EAST = -90, [-180,180]
        imuAngle = HelperFunctions.unity2vehDynCoord(-imuAngle);

    }


}
}