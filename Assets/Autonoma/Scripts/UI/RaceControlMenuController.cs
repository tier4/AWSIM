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
using UnityEngine.UI;
using UnityEngine.InputSystem;
using TMPro;
using Autonoma;

public class RaceControlMenuController : MonoBehaviour
{
    public List<GameObject> rosCars;
    public TMP_Dropdown trackFlagDropdown;     
    public TMP_Dropdown vehFlagDropdown;
    public int[] track_flag_vec = {3,9,1};
    public int[] veh_flag_vec = {0,25,7,34,4,33};
    bool initialized = false;
    void Start()
    {
        trackFlagDropdown.onValueChanged.AddListener(delegate { trackFlagChanged(); } );

        trackFlagChanged();

        vehFlagDropdown.onValueChanged.AddListener(delegate { vehFlagChanged(); } );

        vehFlagChanged();

    }

    void Update()
    {
        if (!initialized && rosCars.Count > 0)
        {
            vehFlagChanged();
            trackFlagChanged();
            initialized = true;
        }
    }
    void OnDisable()
    {
        initialized = false;
    }

    void trackFlagChanged()
    {
        int idx = trackFlagDropdown.value;
        foreach(GameObject car in rosCars)
        {
            RaceControlData raceControl = car.transform.Find("URDF").Find("base_link").Find("Vehicle Sensors").Find("Race Control").GetComponent<RaceControlData>();
            raceControl.rc.TrackFlag = (byte)track_flag_vec[idx];
        }
    }
    void vehFlagChanged()
    {
        // All cars get same vehicle flag
        int idx = vehFlagDropdown.value;
        foreach(GameObject car in rosCars)
        {
            RaceControlData raceControl = car.transform.Find("URDF").Find("base_link").Find("Vehicle Sensors").Find("Race Control").GetComponent<RaceControlData>();
            raceControl.rc.VehicleFlag = (byte)veh_flag_vec[idx];
        }    
    }
}
