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

[CreateAssetMenu]
public class VehicleParameters : ScriptableObject
{
    // ------------ Vehicle Main Dimensions and Mass properties
    public float mass;// = 790f; // mass
    public Vector3 centerOfMass; //= new Vector3(0f,0.2f,0f);
    public Vector3 Inertia;// = new Vector3(800f,1100f,265f); 
    public float lf;// = 1.7f; 
    public float lr;// = -1.3f; // distance of rear axle to cg 
    public float twf;//  = (1.93f-0.254f)/2; // front half track width
    public float twr;//  = (1.93f-0.35f)/2; // rear half track width
    public Vector3 w1pos;
    public Vector3 w2pos;
    public Vector3 w3pos;
    public Vector3 w4pos;
    // ----- Steering kinematics and actuator dynamics
    public float maxSteeringAngle;// = 200f;
    public float steeringRatio;// = -19.5f; // ratio between input steering angle to wheel angle. whats the max input angle ? 
    public int steeringDelay;
    public float steeringDelaySec;// = 0.04f; // s
    public float steeringBandwidth;// = 1.0f; // lpf freq
    public float steeringRate;// = 60.0f; // deg/s at the wheel
    //----------- Engine and wheel rotation Dynamics -----------------
    // 0 4000 8000 , 0 0.5 1
    //public float[,]  engineMap= {{0,200,400},{0,200,400},{0,0,0}};
    public int numGears;//  = 6;
    public float[] gearRatio;// = { 3f * 35f / 12f, 3f * 30f / 16f, 3f * 29f / 21f, 3f * 29f / 26f, 3f * 24f / 25f, 3f * 24f / 27f };
    public double[] enginePoly;// = {-4.56821e-05,0.489643,-754.247};
    public float throttleBandwidth;// = 1.0f; // lpf freq
    public float throttleRate;// = 60.0f; // unit/s 
    public float torqueRate;// = 150.0f; // Nm/s  
    public int numPointsThrottleMap;// = 3;
    public float[] throttleMapInput;// = {0f , 0.5f, 1f};
    public float[] throttleMapOutput;// = {0f , 0.5f, 1f};
    public float maxEngineRpm;// = 7200f;
    public float minEngineMapRpm;// = 2400f;
    public float engineFrictionTorque;// = 30f;
    public float engineInertia;
    public float frontDifferentialDamping;
    public float rearDifferentialDamping;
    public bool rearSolidAxle;
//---------- Brake Dynamics ----------------------
    public int brakeDelay; // num discrete steps
    public float brakeDelaySec;// = 0.04f; // s
    public float maxBrakeKpa;// = 9000f; // max brake in kPa,
    public float brakeKpaToNm;// = 0.54f;
    public float brakeBias;// = 0.45f; // how much of the total torque goes to the front
    public float brakeBandwidth;// = 1.0f; // lpf freq
    public float brakeRate; // kpa/s at the wheel
    // --------------- Suspension -----------------------
    public float kArbFront;
    public float kArbRear;
    public float kSpring;// = 200000.0f; // [N/m] carrying 800kg car with 10cm deflection
    public float cDamper;// = 4000.0f; //8855 [Ns/m] for zeta = 0.707 = c/(2*sqrt(km))
    public float lSpring;// = 0.3f; // [m]

    public TyreParameters frontTyreParams,rearTyreParams;
    public float tAmb,tTrack;

    // ------------------Aerodynamics --------------------------
    public float Af;// = 1; // [m^2] Frontal Area
    public float ClF;// = 0.5f; // downforce coef. at the front axle
    public float ClR;// = 0.5f*1.6f/1.3f; // downforce coef. at the rear axle
    public Vector3 frontDownforcePos;
    public Vector3 rearDownforcePos;
    public float Cd;// = 0.81f; 
    public Vector3 dragForcePos;// = new Vector3(0f,0.2f,0f); // [m] center of pressure for the drag force, above the origin point of the chassis

    public void calcDepVars()
    {
        brakeDelay = (int)Mathf.RoundToInt(brakeDelaySec/Time.fixedDeltaTime);
        steeringDelay = (int) Mathf.RoundToInt(steeringDelaySec/Time.fixedDeltaTime);
        brakeRate = maxBrakeKpa*60.0f;
        frontDownforcePos = new Vector3(0f,0f,lf); 
        rearDownforcePos = new Vector3(0f,0f,lr);
        w1pos = new Vector3(-twf, lSpring + frontTyreParams.tyreRadius, lf);
        w2pos = new Vector3(twf, lSpring + frontTyreParams.tyreRadius, lf);
        w3pos = new Vector3(-twr, lSpring + rearTyreParams.tyreRadius, lr);
        w4pos = new Vector3(twr, lSpring + rearTyreParams.tyreRadius, lr);
    }
}

