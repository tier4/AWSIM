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

public class TrackInfoFiller : MonoBehaviour
{
    public TrackParams trackParams;
    public bool record;
    public bool clear;
    public Vector3 prevPos;
    void Start()
    {
        trackParams = GameManager.Instance.Settings.myTrackParams;
    }

    void ClearInfo()
    {
        trackParams.trackInfo.innerX.Clear();
        trackParams.trackInfo.innerZ.Clear();
        trackParams.trackInfo.innerY.Clear();
        trackParams.trackInfo.heading.Clear();
        
        prevPos = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        if (clear)
        {
            ClearInfo();
            Debug.Log("Track Info Cleared");
            clear = false;
        }
        if (record &&  Vector3.Distance(transform.position , prevPos) >=5f)
        {
            trackParams.trackInfo.innerX.Add(transform.position.x);
            trackParams.trackInfo.innerZ.Add(transform.position.z);
            trackParams.trackInfo.innerY.Add(transform.position.y);
            trackParams.trackInfo.heading.Add(transform.eulerAngles.y);
            prevPos = transform.position;
        }        
    }
}
