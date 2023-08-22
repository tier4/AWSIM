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

using System;
using UnityEngine;

namespace VehicleDynamics{
public class HelperFunctions
{
    public static float MathMod(float a, float b) 
    {
        while (a < 0f)
        {
            a+=b;
        }
        return a % b;
    }
    public static double myLerp(float p0, float p1, float ratio)
    {
        return p0 + (p1-p0)*Mathf.Clamp(ratio,0.0f,1.0f);
    }

    public static Vector3 unity2vehDynCoord(Vector3 unityCoord)
    {   // transformation from LHS unity coordinate system to RHS vehicle dynamics coordinate system(x front, y left, z up of the vehicle)

        Vector3 rhsCoord = new Vector3(unityCoord[0],unityCoord[2],unityCoord[1]);
        Vector3 vehDynCoord = new Vector3(rhsCoord[1],-rhsCoord[0],rhsCoord[2]);
        return vehDynCoord;
    }
    public static Vector3 unity2enu(Vector3 unityCoord)
    {   // transformation from LHS unity coordinate system to ENU coordinate system(x = EAST,z = North,y = UP)

        Vector3 enuCoord = new Vector3(unityCoord[0],unityCoord[2],unityCoord[1]);
        return enuCoord;
    }
    public static double rad2deg(double radians)
    {
        double degrees = (double)(180 / Math.PI) * radians;
        return (degrees);
    }

    public static double deg2rad(double degrees)
    {
        double radians = (double)(degrees / (180.0 / Math.PI));
        return (radians);
    }

    public static float lowPassFirstOrder(float u, float yPrev , float freq)
    {
        // euler forward discretization; y  = yPrev + ydot*dt;
        float omega = 2*Mathf.PI*freq; 
        float yDot = (-omega*yPrev+omega*u);
        float y = yPrev + yDot*Time.fixedDeltaTime;
        return y;
    }

    public static float rateLimit(float yCur, float yPrev , float speed)
    {
        float yLimited;
        if ( Mathf.Abs(yCur-yPrev)/Time.fixedDeltaTime > speed )
            yLimited = yCur>yPrev? yPrev + speed*Time.fixedDeltaTime : yPrev - speed*Time.fixedDeltaTime;
        else
            yLimited = yCur;
    
        return yLimited;
    }

    public static float rateLimitUpdate(float yCur, float yPrev , float speed)
    {
        float yLimited;
        if ( Mathf.Abs(yCur-yPrev)/Time.deltaTime > speed )
            yLimited = yCur>yPrev? yPrev + speed*Time.deltaTime : yPrev - speed*Time.deltaTime;
        else
            yLimited = yCur;
    
        return yLimited;
    }

    public static float[] pureDelay(float input, float[] prevVec, int delay)
    {
        float[] newVec = new float[delay];
        newVec[0] = input; 
        for (int i = 0; i < delay-1; i++)
        {
            newVec[i+1]  = prevVec[i];
        }
        
        return newVec;

    }
    public static float interpolate(float x1, float x2, float y1, float y2, float x) {
        float dydx = (y2 - y1) / (x2 - x1);
        return y1 + (x - x1) * dydx;
    }
    public static float lut1D(int numPoints, float[] tableX, float[] tableY, float inputX)
    {
        if (inputX <= tableX[0])
            return tableY[0];
        else if (inputX >= tableX[numPoints-1])
            return tableY[numPoints-1];
        else
        {
            for (int i = 0; i < numPoints; i++){
                if(tableX[i]<inputX && tableX[i+1]>inputX)
                return interpolate(tableX[i],tableX[i+1],tableY[i],tableY[i+1],inputX);
            }
        }
        return float.NaN;
    }

    public static int findCurrIdx(float input, float[] inputVec, int numPoints)
    {
        if (input <= inputVec[0])
            return 0;
        else if (input >= inputVec[numPoints-1])
            return numPoints-1;
        else
        {
            for (int i = 0; i < numPoints; i++){
                if(inputVec[i]<input && input<inputVec[i+1])
                return i;
            }
        }
        return -1;
    }

    public static float lut2D(int numPoints1, int numPoints2, float[] input1Vec, float[] input2Vec, float[,] output, float input1, float input2)
    {
        int currIdx1 = findCurrIdx(input1,input1Vec,numPoints1); // for throttle axis
        int currIdx2 = findCurrIdx(input2,input2Vec,numPoints2); // for rpm axis

        // these 2 are the interpolation to the exact rpm at the throttle values of table throttle points(horizontal interpolation of lower and upper throttle bounds)
        float Tk = interpolate(input2Vec[currIdx2],input2Vec[currIdx2+1],output[currIdx1,currIdx2],output[currIdx1,currIdx2+1],input2);
        float Tk1 = interpolate(input2Vec[currIdx2],input2Vec[currIdx2+1],output[currIdx1+1,currIdx2],output[currIdx1+1,currIdx2+1],input2);
        
        // final interpolation for the exact throttle of the given torques(vertical interpolation)
        float Tf = interpolate(input1Vec[currIdx1],input1Vec[currIdx1+1],Tk,Tk1,input1); 
        
        return Tf;
    }
    public static Rigidbody GetTopParentRigidbody(Transform currentTransform)
    {
        // Check if there is no parent or if the parent has no Rigidbody component
        if (currentTransform.parent != null && currentTransform.parent.GetComponent<Rigidbody>() != null)
        {
            // Return the Rigidbody component of the current game object
            return currentTransform.parent.GetComponent<Rigidbody>();
        }
        // Call the function recursively with the parent transform
        return GetTopParentRigidbody(currentTransform.parent);
    }

    public static T GetParentComponent<T>(Transform currentTransform) where T : Component
    {
        // Check if there is no parent or if the parent has no T component
        if (currentTransform.parent != null && currentTransform.parent.GetComponent<T>() != null)
        {
            // Return the T component of the current game object
            return currentTransform.parent.GetComponent<T>();
        }
        // Call the function recursively with the parent transform
        return GetParentComponent<T>(currentTransform.parent);
    }

    public static Transform GetParentTransform(Transform currentTransform)
    {
        // Check if there is no parent or if the parent has no T component
        if (currentTransform.parent != null)
        {
            // Return the T component of the current game object
            return GetParentTransform(currentTransform.parent);
        }
        // Call the function recursively with the parent transform
        return currentTransform;
    }
}
}