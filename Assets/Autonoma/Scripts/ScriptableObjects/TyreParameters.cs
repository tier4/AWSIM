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
public class TyreParameters : ScriptableObject
{
    public float FzNom;
    public float Dy;
    public float Dy2;
    public float Cy;
    public float syPeak;
    public float relaxLenY; //m
    public float Dx;
    public float Dx2;
    public float Cx;
    public float sxPeak;
    public float relaxLenX;
    public float rollResForce;
    public float wheelInertia;
    public float tyreRadius;
    public float p1,p2,p3,p4,p5,p6,p7,mT,cT,hT,ACp;
    public int numPointsFrictionMap;
    public float[] thermalFrictionMapInput;// = {0f , 0.5f, 1f};
    public float[] thermalFrictionMapOutput;// = {0f , 0.5f, 1f};

    public void calcDepVars()
    {

    }
}

