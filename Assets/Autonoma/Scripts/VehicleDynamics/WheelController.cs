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


public class WheelController : MonoBehaviour
{       
    public Rigidbody carBody;
    public CarController carController;
    public TyreParameters axleTyreParams;
    public GameObject mesh;
    public WheelController otherSideWheel;
    public bool wheelFL;
    public bool wheelFR;
    public bool wheelRL;
    public bool wheelRR;
    public float hitDistance = 0.55f;
    public float currLength = 0.24f;
    private float lastLength;
    public float springDeflection, springVelocity;
    public float springForce, damperForce,arbForce, Fx, Fy, Fz;
    public float sy, sx, S , syDir, sxDir, tanSy, tanSyPrev, sxPrev;
    public Vector3 wheelVelocity;
    public bool isHit;
    private Vector3 Ftotal;
    public float wheelStAngle, omega, omegaDot, wheelAngle, driveTorque;
    public float currTyreTemp,q1,q2,q3,q4,thermalScaling;
    public float DxEffective,DyEffective;
    void setWheelPos()
    {
        if (wheelFL)
            transform.localPosition = carController.vehicleParams.w1pos;
        if (wheelFR)
            transform.localPosition = carController.vehicleParams.w2pos;
        if (wheelRL)
            transform.localPosition = carController.vehicleParams.w3pos;
        if (wheelRR)
            transform.localPosition = carController.vehicleParams.w4pos;  
    }
    void getSteering()
    {
        if (wheelFL || wheelFR)
        {
            wheelStAngle = carController.steerAngleApplied;
        }
        else
        {
            wheelStAngle = 0f;
        }
    }
    void calcOmega()
    {   
        float brakeBias; 
        float wheelInertia;
        if (wheelFL || wheelFR)
        {
            wheelInertia = axleTyreParams.wheelInertia;
            brakeBias = carController.vehicleParams.brakeBias;
        }
        else
        {   
            float gr = carController.vehicleParams.gearRatio[carController.gear-1];
            wheelInertia = axleTyreParams.wheelInertia + 0.5f*gr*gr*carController.vehicleParams.engineInertia;
            brakeBias = 1f-carController.vehicleParams.brakeBias;
        }
        omegaDot = (driveTorque - 0.5f*carController.TBrake*brakeBias*Mathf.Sign(omega)
                           - Fx*axleTyreParams.tyreRadius
                           - axleTyreParams.rollResForce*axleTyreParams.tyreRadius*Mathf.Sign(omega) )/wheelInertia;

        if (Mathf.Abs(omega) < 0.5f && carController.TBrake > 10f && driveTorque < 1f)
        {
            omega = 0;
        }
        else
        {
            omega += omegaDot*Time.fixedDeltaTime;
        }

        if (wheelRL || wheelRR)
        {
            omega = Mathf.Clamp(omega,-50f,carController.vehicleParams.maxEngineRpm*(2*Mathf.PI)/60f/carController.vehicleParams.gearRatio[carController.gear-1]);
        }
    }
    void calcArbForce()
    {
        float arbStiffness;
        if (wheelFL || wheelFR)
        {
            arbStiffness = carController.vehicleParams.kArbFront;
        }
        else
        {
            arbStiffness = carController.vehicleParams.kArbRear;
        }
        arbForce = arbStiffness * (this.springDeflection - otherSideWheel.springDeflection);
    }

    void calcFz()
    {
        lastLength = currLength;
        currLength = hitDistance -  axleTyreParams.tyreRadius;
        
        springDeflection = carController.vehicleParams.lSpring - currLength;
        springForce =  carController.vehicleParams.kSpring * springDeflection;

        springVelocity = (lastLength - currLength) / Time.fixedDeltaTime;
        damperForce =  carController.vehicleParams.cDamper * springVelocity;
        calcArbForce();
        Fz = springForce + damperForce + arbForce; 

        Fz = Mathf.Clamp(Fz,0f,Fz);
    }

    void calcSySx()
    {
        // calculate non relax len versions of slips for validation purposes
        syDir = Mathf.Abs(wheelVelocity.z ) > 0.1f ? Mathf.Atan(-wheelVelocity.x / Mathf.Abs(wheelVelocity.z ) ) : syDir;
        sxDir = Mathf.Abs(wheelVelocity.z ) > 0.1f ? (omega*axleTyreParams.tyreRadius-wheelVelocity.z)/Mathf.Abs(wheelVelocity.z ) : sxDir ;
       
        float tansyDot = (-wheelVelocity.x - Mathf.Abs(wheelVelocity.z)*tanSyPrev) /axleTyreParams.relaxLenY;
        tanSy = tanSyPrev + tansyDot*Time.fixedDeltaTime;
        sy = Mathf.Atan(tanSy);
        
        float uSat = Mathf.Max(Mathf.Abs(wheelVelocity.z),1);
        float sxDot = (omega*axleTyreParams.tyreRadius - wheelVelocity.z - Mathf.Abs(wheelVelocity.z)*sxPrev )/axleTyreParams.relaxLenX ;
        sx = sxPrev + sxDot*Time.fixedDeltaTime;
        sx = Mathf.Clamp(sx,-1f,1f);

        tanSyPrev = tanSy;
        sxPrev = sx;
    }
    public Vector3 calcTyreForcesNonlinear()
    {
        S = Mathf.Sqrt(Mathf.Pow((sy),2) + Mathf.Pow(sx,2));
        float sy_S = sy/Mathf.Max(S,0.0001f);
        float sx_S = sx/Mathf.Max(S,0.0001f);

        float By = Mathf.Tan(Mathf.PI/(2f*axleTyreParams.Cy))/axleTyreParams.syPeak;
        float Bx = Mathf.Tan(Mathf.PI/(2f*axleTyreParams.Cx))/axleTyreParams.sxPeak;

        calcThermalScaling();
        float FzLoad = Mathf.Clamp(Fz,axleTyreParams.FzNom/3f,axleTyreParams.FzNom*3f);
        DyEffective = axleTyreParams.Dy + axleTyreParams.Dy2*(FzLoad-axleTyreParams.FzNom)/axleTyreParams.FzNom;
        DxEffective = axleTyreParams.Dx + axleTyreParams.Dx2*(FzLoad-axleTyreParams.FzNom)/axleTyreParams.FzNom;
        //DxEffective = Mathf.Clamp(DxEffective,DxEffective/2f,DxEffective*2f);
        //DyEffective = Mathf.Clamp(DyEffective,DyEffective/2f,DyEffective*2f);

        Fy = thermalScaling*Fz*sy_S*DyEffective*Mathf.Sin(axleTyreParams.Cy*Mathf.Atan(By*S));
        Fx = thermalScaling*Fz*sx_S*DxEffective*Mathf.Sin(axleTyreParams.Cx*Mathf.Atan(Bx*S));
   
        //Fx -= axleTyreParams.rollResForce * Mathf.Sign(omega);
        
        if (float.IsNaN(Fx) ) Fx = 0f; 
        if (float.IsNaN(Fy) ) Fy = 0f; 

        Vector3 localForceTotal = new Vector3(0f,0f,0f);
        localForceTotal = Fz  * transform.up + Fx * transform.forward + Fy * transform.right;
        return localForceTotal;
    }

    void calcThermalScaling()
    {
        if (GameManager.Instance.Settings.myVehSetup.IsThermalTyre)
        {
            thermalScaling = HelperFunctions.lut1D(axleTyreParams.numPointsFrictionMap,
                                axleTyreParams.thermalFrictionMapInput, axleTyreParams.thermalFrictionMapOutput, currTyreTemp);
        }
        else
        {
            thermalScaling = 1.0f;
        }
    }

    void calcTyreTemp()
    {
        // Compute the heat generation
        q1 = axleTyreParams.p1 * Mathf.Abs(wheelVelocity.z) * (Mathf.Abs(Fx * sx) + Mathf.Abs(Fy * Mathf.Tan(sy)));
            
        q2 = Mathf.Abs(wheelVelocity.z) * (axleTyreParams.p2 * Mathf.Abs(Fx) + axleTyreParams.p3 * Mathf.Abs(Fy) + axleTyreParams.p4 * Mathf.Abs(Fz));

        // Compute the heat dissipation
        q3 = axleTyreParams.p5 * Mathf.Pow(Mathf.Clamp(Mathf.Abs(wheelVelocity.z),1f,Mathf.Abs(wheelVelocity.z)),axleTyreParams.p6) * (currTyreTemp - carController.vehicleParams.tAmb);

        // Compute the heat transfer to the track
        float isMoving = Mathf.Abs(wheelVelocity.z) > 5f ? 1f : 0f;
        q4 =  axleTyreParams.hT * axleTyreParams.ACp * (currTyreTemp - carController.vehicleParams.tTrack);

        // Compute the rate of change of tyre temperature
        float dT_s = (q1 + q2 - q3 - q4) / (axleTyreParams.mT * axleTyreParams.cT);

        currTyreTemp += dT_s*Time.fixedDeltaTime;

        currTyreTemp = Mathf.Clamp(currTyreTemp, 0.0f, 199.0f);
    }
    void Start()
    {   
        currTyreTemp = carController.vehicleParams.tAmb;
        setWheelPos();
        wheelAngle = 0.0f;
    }
    void FixedUpdate() 
    {   
        getSteering();
        calcOmega();
        transform.localRotation = Quaternion.Euler(Vector3.up * wheelStAngle);
        isHit = Physics.Raycast( transform.position, -transform.up, out RaycastHit hit,  carController.vehicleParams.lSpring + axleTyreParams.tyreRadius);       
        if (isHit)
        {   
            wheelVelocity = transform.InverseTransformDirection(carBody.GetPointVelocity(hit.point));
            hitDistance = hit.distance;
            calcFz();
            calcSySx();
            Ftotal = calcTyreForcesNonlinear();
            
        }
        else
        {
            Ftotal = new Vector3(0f,0f,0f);
        }
        carBody.AddForceAtPosition(Ftotal, hit.point);
        calcTyreTemp();
    }
    void Update()
    {
        bool isHit2 = Physics.Raycast(transform.position, -transform.up, out RaycastHit hit,  carController.vehicleParams.lSpring + axleTyreParams.tyreRadius);
        float tyrePos = -carController.vehicleParams.lSpring;
        if (isHit2)
        {
            Debug.DrawRay(hit.point, transform.forward * Fx * 0.001f, Color.red);
            Debug.DrawRay(hit.point, transform.right * Fy * 0.001f, Color.blue);
            Debug.DrawRay(hit.point, transform.up * Fz * 0.001f, Color.green);
            tyrePos = -currLength;
        }
        // update viusals of the mesh for wheel rotation and susp deflection
        mesh.transform.localPosition = new Vector3(0, tyrePos, 0); 
        mesh.transform.Rotate(Vector3.right, Time.deltaTime*180/(Mathf.PI)*omega);
    }
}
