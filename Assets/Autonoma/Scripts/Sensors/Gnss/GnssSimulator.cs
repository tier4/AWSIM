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

public class GnssSimulator : MonoBehaviour
{
    public Vector3 antennaVelGlobal;
    public Vector3 antennaPosGlobal;
    public Rigidbody rb;
    public double[] llh = {0.0,0.0,0.0};
    public double lat,lon,height;
    public double lat0,lon0,h0;    
    public float vE, vN, vU;
    void Start() 
    {
        rb = HelperFunctions.GetParentComponent<Rigidbody>(transform);
        try
        {
            lat0 = GameManager.Instance.Settings.myTrackParams.LAT_ORIGIN;
            lon0 = GameManager.Instance.Settings.myTrackParams.LON_ORIGIN;
            h0 = GameManager.Instance.Settings.myTrackParams.HEIGHT_ORIGIN;
        }
        catch 
        {
            Debug.Log("lat0 lon0 not defined!");
        }

    } 
    void FixedUpdate()
    {   
        // E N U , RPY
        antennaPosGlobal = HelperFunctions.unity2enu(transform.position);
        llh = Enu2LatLonHeight.calcLatLonHeight(antennaPosGlobal,lat0,lon0,h0);
        lat = llh[0];
        lon = llh[1];
        height = llh[2];
        
        antennaVelGlobal = HelperFunctions.unity2enu( rb.GetPointVelocity( transform.position ) );
        vE = antennaVelGlobal[0];
        vN = antennaVelGlobal[1];
        vU = antennaVelGlobal[2];
    }
}

