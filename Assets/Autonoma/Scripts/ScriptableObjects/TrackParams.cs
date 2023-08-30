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
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "TrackParams", menuName = "Custom/Track Params")]
public class TrackParams : ScriptableObject
{
    public string TrackName;
    //public GameObject prefab;
    public double LAT_ORIGIN = 0.0000000;
    public double LON_ORIGIN = 0.0000000;
    public double HEIGHT_ORIGIN = 0.0;
    public Vector3 trackRotation = new Vector3(0f, 0f, 0f);
    public Vector3 carRotation = new Vector3(0f, 0f, 0f);
    public Vector3 trackScaling = new Vector3 (1f, 1f, 1f);
    public Vector3 spawnDeltaPos = new Vector3 (-10f, 0f, 0f);
    public List<Vector3> carSpawnPositions = new List<Vector3>() 
    { 
        new Vector3(0f, 0f, 0f)
    };
    public TrackInfo trackInfo;
    public void populateStartPositions()
    {
        for (int i = 0; i < 51; i++)
        {
            Vector3 newPos = carSpawnPositions[i] + spawnDeltaPos;
            carSpawnPositions.Add(newPos);
        }
    }
}