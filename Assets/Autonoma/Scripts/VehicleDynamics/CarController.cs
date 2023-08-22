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

public class CarController : MonoBehaviour
{
    public VehicleParameters vehicleParams;
    public Rigidbody carBody;
    public float throttleCmd, brakeCmd, steerAngleCmd; // controller inputs
    public float steerAngleApplied, steerAngleAppliedPrev; // after the actuator dyn
    public float brakeApplied, brakeAppliedPrev ,thrApplied, thrAplliedPrev;  // after the actuator dyn
    private float[] steerAngleCmdBuf;
    private float[] steerAngleCmdBufPrev;
    private float[] brakeCmdBuf;
    private float[] brakeCmdBufPrev;
    public bool gearUp, gearDown , gearUpPrev, gearDownPrev;
    public float omegaRR,omega1,omega2;
    public int gear = 1;
    public float TEngine,TEnginePrev,rpmEngine;
    public float TAxle,TBrake;
    public Vector3 frontDownforce,frontDownforceGlobal;
    public Vector3 rearDownforce,rearDownforceGlobal;
    public Vector3 dragForce,dragForceGlobal;
    public Vector3 V;
    public bool physicalActuator = false;
    public VehicleState vehicleState;
    public Powertrain powertrain;
    void getState()
    {
        // take values from unity system and transform into VD coords
        V = HelperFunctions.unity2vehDynCoord( transform.InverseTransformDirection(carBody.velocity) );

        Vector3 pose = HelperFunctions.unity2vehDynCoord(-transform.eulerAngles);
        pose.x = Mathf.Abs(pose.x) > 180f ? pose.x - Mathf.Sign(pose.x)*360f : pose.x;
        pose.y = Mathf.Abs(pose.y) > 180f ? pose.y - Mathf.Sign(pose.y)*360f : pose.y;
        pose.z = Mathf.Abs(pose.z) > 180f ? pose.z - Mathf.Sign(pose.z)*360f : pose.z;
        
        Vector3 localAngularvelocity = transform.InverseTransformDirection(carBody.angularVelocity);
        Vector3 carAngularVel = HelperFunctions.unity2vehDynCoord(-localAngularvelocity);
        Vector3 position = HelperFunctions.unity2vehDynCoord(transform.position);
        vehicleState.pos[0] = position.x;
        vehicleState.pos[1] = position.y;
        vehicleState.pos[2] = position.z;
        vehicleState.V[0] = V.x;
        vehicleState.V[1] = V.y;
        vehicleState.V[2] = V.z;
        vehicleState.rollRate = carAngularVel.x;
        vehicleState.pitchRate = carAngularVel.y;
        vehicleState.yawRate = carAngularVel.z;
        vehicleState.roll = pose.x;
        vehicleState.pitch = pose.y;
        vehicleState.yaw = pose.z;
        vehicleState.omega[0] = omega1;
        vehicleState.omega[1] = omega2;
        vehicleState.omega[2] = omegaRR;
        vehicleState.omega[3] = omegaRR;    
    }

    void calcWheelStAngle()
    {   
        if (physicalActuator)
        {
            steerAngleCmdBufPrev = steerAngleCmdBuf;
            steerAngleCmdBuf = HelperFunctions.pureDelay(steerAngleCmd,steerAngleCmdBufPrev, vehicleParams.steeringDelay);
            steerAngleApplied = steerAngleCmdBuf[vehicleParams.steeringDelay-1];
            steerAngleApplied = HelperFunctions.lowPassFirstOrder(steerAngleApplied,steerAngleAppliedPrev,vehicleParams.steeringBandwidth);
            steerAngleApplied = HelperFunctions.rateLimit(steerAngleApplied, steerAngleAppliedPrev , Mathf.Abs(vehicleParams.steeringRate/vehicleParams.steeringRatio));
            steerAngleAppliedPrev = steerAngleApplied;
        }
        else
        {
            steerAngleApplied = steerAngleCmd;
        }
    }

    void calcBrakeTorque()
    {   
        if (physicalActuator)
        {
            brakeCmdBufPrev = brakeCmdBuf;
            brakeCmdBuf = HelperFunctions.pureDelay(brakeCmd,brakeCmdBufPrev, vehicleParams.brakeDelay); // buffer the incoming commands
            brakeApplied = brakeCmdBuf[vehicleParams.brakeDelay-1]; // select the latest of the buffer for the delay
            brakeApplied = HelperFunctions.lowPassFirstOrder(brakeApplied,brakeAppliedPrev, vehicleParams.brakeBandwidth);
            brakeApplied = HelperFunctions.rateLimit(brakeApplied, brakeAppliedPrev , vehicleParams.brakeRate);
            brakeAppliedPrev = brakeApplied;
            TBrake = brakeApplied * vehicleParams.brakeKpaToNm;
        }
        else
        {   brakeApplied = brakeCmd;
            TBrake = brakeCmd * vehicleParams.brakeKpaToNm;
        }
    }

    void gearShifts()
    {
        if (gearUp && !gearUpPrev && gear < vehicleParams.numGears)
            gear++;
        if (gearDown && !gearDownPrev && gear > 0)
            gear--;  

        gear = Mathf.Clamp(gear,1,vehicleParams.numGears);    

        gearUpPrev = gearUp;
        gearDownPrev = gearDown;
    }

    void calcEngineTorque()
    {        
        //TEngine = throttleCmd * HelperFunctions.lut1D(VehParams.engineMapLen,VehParams.engineMapRevs,VehParams.engineMapTorque,rpmEngine);
        float saturatedRpmEngine = Mathf.Clamp(rpmEngine,vehicleParams.minEngineMapRpm,vehicleParams.maxEngineRpm);
        thrApplied = HelperFunctions.lowPassFirstOrder(throttleCmd,thrAplliedPrev, vehicleParams.throttleBandwidth);
        thrApplied = HelperFunctions.rateLimit(thrApplied, thrAplliedPrev , vehicleParams.throttleRate);
        thrAplliedPrev = thrApplied;

        float torquePercentage = HelperFunctions.lut1D(vehicleParams.numPointsThrottleMap,
                                vehicleParams.throttleMapInput, vehicleParams.throttleMapOutput, thrApplied);

        TEngine = torquePercentage *( (float)(vehicleParams.enginePoly[0]*Mathf.Pow(saturatedRpmEngine,2) 
                + vehicleParams.enginePoly[1]*saturatedRpmEngine + vehicleParams.enginePoly[2]) + vehicleParams.engineFrictionTorque);
        if (rpmEngine > 1050)
        {
            TEngine = TEngine - vehicleParams.engineFrictionTorque;
        }
        if (TEngine > TEnginePrev )
        {
            TEngine = HelperFunctions.rateLimit(TEngine, TEnginePrev , vehicleParams.torqueRate);
        }
        TAxle = TEngine * vehicleParams.gearRatio[gear-1];
        TEnginePrev= TEngine;
    }
    void applyAeroForces()
    {
        float aeroForce = -0.6f*vehicleParams.Af*V.x*V.x;
        frontDownforce.y  = aeroForce*vehicleParams.ClF;
        rearDownforce.y = aeroForce*vehicleParams.ClR;
        dragForce.z = aeroForce*vehicleParams.Cd*Mathf.Sign(V.x);
        frontDownforceGlobal = transform.TransformDirection(frontDownforce);
        rearDownforceGlobal  = transform.TransformDirection(rearDownforce);
        dragForceGlobal = transform.TransformDirection(dragForce);

        carBody.AddForceAtPosition(frontDownforceGlobal, transform.TransformPoint(vehicleParams.frontDownforcePos));
        carBody.AddForceAtPosition(rearDownforceGlobal, transform.TransformPoint(vehicleParams.rearDownforcePos));
        carBody.AddForceAtPosition(dragForceGlobal, transform.TransformPoint(vehicleParams.dragForcePos));
    } 
    
    void Start()
    {
        UpdateVehicleParamsFromMenu();

        initializeBuffers();
        carBody = GetComponent<Rigidbody>();
        carBody.mass = vehicleParams.mass;
        carBody.inertiaTensor = vehicleParams.Inertia;
        carBody.centerOfMass = vehicleParams.centerOfMass;
    }

    void UpdateVehicleParamsFromMenu()
    {
        vehicleParams.brakeKpaToNm = GameManager.Instance.Settings.myVehSetup.BrakeConstant;
        vehicleParams.kArbFront = GameManager.Instance.Settings.myVehSetup.FrontRollBarRate;
        vehicleParams.kArbRear = GameManager.Instance.Settings.myVehSetup.RearRollBarRate;
        vehicleParams.maxSteeringAngle = (GameManager.Instance.Settings.myVehSetup.MaxSteeringAngle > 0) ? GameManager.Instance.Settings.myVehSetup.MaxSteeringAngle : 200f;
        vehicleParams.rearSolidAxle = !GameManager.Instance.Settings.myVehSetup.IsLSD;
        vehicleParams.steeringBandwidth = (GameManager.Instance.Settings.myVehSetup.SteeringBW > 0) ? GameManager.Instance.Settings.myVehSetup.SteeringBW : 5f;
        vehicleParams.steeringDelaySec = (GameManager.Instance.Settings.myVehSetup.SteeringDelay > 0) ? GameManager.Instance.Settings.myVehSetup.SteeringDelay : 0.01f;
        vehicleParams.steeringRate = (GameManager.Instance.Settings.myVehSetup.MaxSteeringRate > 0) ? GameManager.Instance.Settings.myVehSetup.MaxSteeringRate : 360f;
        vehicleParams.steeringRatio = -1f*Mathf.Abs(GameManager.Instance.Settings.myVehSetup.SteeringRatio);
        vehicleParams.tAmb = GameManager.Instance.Settings.myVehSetup.AmbientTemp;
        vehicleParams.tTrack = GameManager.Instance.Settings.myVehSetup.TrackTemp;
        physicalActuator = !GameManager.Instance.Settings.myVehSetup.IsIdealSteering;
        vehicleParams.calcDepVars();
    }

    void initializeBuffers()
    {
        steerAngleCmdBuf = new float[vehicleParams.steeringDelay];
        steerAngleCmdBufPrev = new float[vehicleParams.steeringDelay];
        brakeCmdBuf = new float[vehicleParams.brakeDelay];
        brakeCmdBufPrev = new float[vehicleParams.brakeDelay];
    }
    
    void Update() 
    {         
        Debug.DrawRay(transform.TransformPoint(vehicleParams.frontDownforcePos), frontDownforceGlobal*0.001f, Color.yellow);
        Debug.DrawRay(transform.TransformPoint(vehicleParams.rearDownforcePos), rearDownforceGlobal*0.001f, Color.red);
        Debug.DrawRay(transform.TransformPoint(vehicleParams.dragForcePos), dragForceGlobal*0.001f, Color.green);
    }
    
    void FixedUpdate()
    {
        getState();
        gearShifts();
        calcWheelStAngle();
        calcBrakeTorque();
        calcEngineTorque();
        applyAeroForces();
    }
}
