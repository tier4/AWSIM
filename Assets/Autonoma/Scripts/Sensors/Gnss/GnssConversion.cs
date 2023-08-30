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
using System;
public static class LatLonHeight2Enu
{
    static double earth_semimajor = 6378137;
    static double earth_eccentricity = 0.00669438;
    static double deg2rad = (double)(1 / (180.0 / Math.PI));

    public static double[] calcEnu(double lat, double lon, double h, double lat0,double lon0,double h0)
    {
        double clatRef = Math.Cos(lat0  * deg2rad);
        double clonRef = Math.Cos(lon0  * deg2rad);
        double slatRef = Math.Sin(lat0   * deg2rad);
        double slonRef = Math.Sin(lon0  * deg2rad);
        double clat    = Math.Cos(lat  * deg2rad);
        double clon    = Math.Cos(lon * deg2rad);
        double slat    = Math.Sin(lat  * deg2rad);
        double slon    = Math.Sin(lon * deg2rad);

        double r0Ref = earth_semimajor / (Math.Sqrt((1.0 - earth_eccentricity * slatRef * slatRef)));
        double[] ecefRef = new double[3];
        ecefRef[0] = (h0 + r0Ref) * clatRef * clonRef;
        ecefRef[1] = (h0 + r0Ref) * clatRef * slonRef;
        ecefRef[2] = (h0 + r0Ref * (1.0 - earth_eccentricity)) * slatRef;  

        double r0 = earth_semimajor / (Math.Sqrt((1.0 - earth_eccentricity * slat * slat)));
        double[] dECEF = new double[3];

        dECEF[0] = (h + r0) * clat * clon - ecefRef[0];              
        dECEF[1] = (h + r0) * clat * slon - ecefRef[1];                
        dECEF[2] = (h + r0 * (1.0 - earth_eccentricity)) * slat - ecefRef[2]; 

        double[,] R = new double[3,3]
        { {-slonRef, clonRef, 0.0},
          { -slatRef * clonRef, -slatRef * slonRef, clatRef},
          {clatRef * clonRef,  clatRef * slonRef, slatRef} };
        
        double[] enu = new double[3];
        for (int row = 0; row<3; row++)
        {   
            enu[row] = 0;
            for (int col = 0; col<3; col++)
            {
                enu[row] += R[row,col]*dECEF[col];
            }
        }
        return enu;
    }
}
public static class Enu2LatLonHeight
{
    public static double h0; 
    public static double lat0rad;
    public static double lon0rad;
    private static double earth_semimajor = 6378137;
    private static double earth_eccentricity = 0.00669438;
    private static double earth_semiminor = 6356752.3;
    private static double earth_eccen2 = earth_eccentricity / (1 - earth_eccentricity);
    private static double flattening = (earth_semimajor - earth_semiminor) / earth_semimajor; 
    private static double lat,lon,height;
    private static double east,north,up;
    private static double u,v,w,x0,y0,z0;
    private static double deg2rad = (double)(1 / (180.0 / Math.PI));
    private static double[] ecef = new double[3];
    private static void geodetic2ecef()
    {
        double N = Math.Pow(earth_semimajor,2)/ ( Math.Sqrt( Math.Pow(earth_semimajor,2) * Math.Pow(Math.Cos(lat0rad),2) 
                                                          +  Math.Pow(earth_semiminor,2) * Math.Pow(Math.Sin(lat0rad),2) ) );

        x0 = (N + h0) * Math.Cos(lat0rad) * Math.Cos(lon0rad);
        y0 = (N + h0) * Math.Cos(lat0rad) * Math.Sin(lon0rad);
        z0 = (N * Math.Pow(earth_semiminor / earth_semimajor,2)+ h0) * Math.Sin(lat0rad);
    }
    private static void enu2uvw()
    {
        double t = Math.Cos(lat0rad) * up - Math.Sin(lat0rad) * north;
        w = Math.Sin(lat0rad) * up + Math.Cos(lat0rad) * north;
        u = Math.Cos(lon0rad) * t - Math.Sin(lon0rad) * east;
        v = Math.Sin(lon0rad) * t + Math.Cos(lon0rad) * east;
    }
    private static void enu2ecef()
    {
        geodetic2ecef();
        enu2uvw();
        ecef[0] = x0 + u;
        ecef[1] = y0 + v;
        ecef[2] = z0 + w;
    }
    private static void ecef2geodetic()
    {
        double r = Math.Sqrt( ecef[0]*ecef[0] + ecef[1]*ecef[1] + ecef[2]*ecef[2] );
        double E = Math.Sqrt( Math.Pow(earth_semimajor,2) - Math.Pow(earth_semiminor,2) );
        double u = Math.Sqrt(0.5 * (r*r - E*E) + 0.5 * Math.Sqrt( Math.Pow(r*r - E*E,2)  + 4 * E * E * ecef[2]*ecef[2]));
        double Q = Math.Sqrt(ecef[0]*ecef[0] + ecef[1]*ecef[1]);
        double huE = Math.Sqrt(u*u + E*E);
        double Beta = 0;
        if (u == 0)
        {
            Beta = ecef[2]>=0 ? Math.PI/2 : -Math.PI/2;
        }
        else
        {
            Beta = Math.Atan(huE / u * ecef[2] / Q);
        }    
    
        double eps = ((earth_semiminor * u - earth_semimajor * huE + E * E) * Math.Sin(Beta)) 
            /(earth_semimajor * huE * 1 / Math.Cos(Beta) - E * E * Math.Cos(Beta) );

        Beta += eps;
        lat = HelperFunctions.rad2deg( Math.Atan(earth_semimajor / earth_semiminor * Math.Tan(Beta)) );
        lon = HelperFunctions.rad2deg( Math.Atan2(ecef[1], ecef[0]) );
        height = Math.Sqrt( Math.Pow( ecef[2] - earth_semiminor * Math.Sin(Beta),2) + Math.Pow(Q - earth_semimajor * Math.Cos(Beta),2) );
        bool inside = Math.Pow(ecef[0] * ecef[0] / earth_semimajor,2) + Math.Pow( ecef[1] * ecef[1] / earth_semimajor,2) + Math.Pow( ecef[2] * ecef[2] / earth_semiminor,2) < 1;
        height = inside ? -height : height; 
    }
    public static double[] calcLatLonHeight(Vector3 pos,double lat0, double lon0, double h0)
    {
        Enu2LatLonHeight.h0  = h0;
        lat0rad = deg2rad * lat0;
        lon0rad = deg2rad * lon0;
        east = (double)pos[0];
        north = (double)pos[1];
        up = (double)pos[2];

        enu2ecef();
        ecef2geodetic(); 
        double[] llh = new double[3];

        llh[0] = lat;
        llh[1] = lon;
        llh[2] = height;
        return llh;
    }

}
