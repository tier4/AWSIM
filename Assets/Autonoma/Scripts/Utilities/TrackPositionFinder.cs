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

public class TrackPositionFinder : MonoBehaviour
{
    public Transform car;
    public TrackInfo trackInfo;
    public float minDist;
    public float currHeading;
    public float lateralError;
    public int minIdx;
    public Vector3 minPos;
    public Vector3 carPos;
    public float dxMin;
    public float dzMin;
    public float maxIdx; 

    void Start()
    {
        trackInfo = GameManager.Instance.Settings.myTrackParams.trackInfo;
        maxIdx = trackInfo.innerX.Count;
    }
    void Update()
    {   
        minDist = 9999;
        carPos = car.position;

        for (int i = 0; i<trackInfo.innerX.Count-1; i++)
        {   
            float dx = carPos.x - (float)trackInfo.innerX[i];
            float dz = carPos.z - (float)trackInfo.innerZ[i];
            float dist = Mathf.Sqrt(dx*dx + dz*dz);
   
            if (dist < minDist)
            {
                minDist = dist;
                minIdx = i;
                dxMin = dx;
                dzMin = dz;
            }
        }

        /*lateralError = dxMin * Mathf.Cos ((float)trackInfo.heading[minIdx] * Mathf.PI/180f) 
                     - dzMin * Mathf.Sin ((float)trackInfo.heading[minIdx] * Mathf.PI/180f);
    */
    }
}
