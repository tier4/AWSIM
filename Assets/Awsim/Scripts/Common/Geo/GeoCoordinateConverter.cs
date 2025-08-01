// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using UnityEngine;

namespace Awsim.Common
{
    /// <summary>
    /// Provide Vector3 to GeoCoordinate conversion.
    /// reference : https://vldb.gsi.go.jp/sokuchi/surveycalc/main.html
    /// </summary>
    public static class GeoCoordinateConverter
    {
        const double _earthSemiMajorAxis = 6378137d;  // [m]
        const double _flatteningInv = 298.257222101d;
        const double _scaleFactor = 0.9999d;

        const double n = 1d / (2d * _flatteningInv - 1d);
        const double n2 = n * n;
        const double n3 = n2 * n;
        const double n4 = n3 * n;
        const double n5 = n4 * n;
        const double n6 = n5 * n;

        static readonly double[] A = new double[6]
        {
            1 + n2 / 4 + n4 / 64,
            -3d / 2d*(n - n3 / 8 - n5 / 64),
            15d / 16d * (n2 - n4 / 4),
            -35d / 48d * (n3 - 5d / 16d * n5),
            315d / 512d * n4,
            -693d / 1280d * n5
        };
        static readonly double A_b = _scaleFactor * _earthSemiMajorAxis / (1 + n) * A[0];

        static readonly double[] beta = new double[6]
        {
            0,
            1d / 2d * n - 2d / 3d * n2 + 37d / 96d * n3 - 1d / 360d * n4 - 81d / 512d * n5,
            1d / 48d * n2 + 1d / 15d * n3 - 437d / 1440d * n4 + 46d / 105d * n5,
            17d / 480d * n3 - 37d / 840d * n4 - 209d / 4480d * n5,
            4397d / 161280d * n4 - 11d / 504d * n5,
            4583d / 161280d * n5
        };

        static readonly double[] delta = new double[7]
        {
            0,
            2d * n - 2d / 3d * n2 - 2d * n3 + 116d / 45d * n4 + 26d / 45d * n5 - 2854d / 675d * n6,
            7d / 3d * n2 - 8d / 5d * n3 - 227d / 45d * n4 + 2704d / 315d * n5 + 2323d / 945d * n6,
            56d / 15d * n3 - 136d / 35d * n4 - 1262d / 105d * n5 + 73814d / 2835d * n6,
            4279d / 630d * n4 - 332d / 35d * n5 - 399572d / 14175d * n6,
            4174d / 315d * n5 - 144838d / 6237d * n6,
            601676d / 22275d * n6
        };

        public static GeoCoordinate Cartesian2Geo(Vector3 cartesian, GeoCoordinate origin)
        {
            double latitude0 = Deg2Rad(origin.Latitude);
            double longitude0 = Deg2Rad(origin.Longitude);

            double S = A[0] * latitude0;
            for (int i = 1; i <= 5; i++)
            {
                S += A[i] * Math.Sin(2 * i * latitude0);
            }
            S = S * (_scaleFactor * _earthSemiMajorAxis) / (1 + n);

            double xi = (-cartesian.x + S) / A_b;
            double eta = cartesian.z / (A_b);

            double xi_d = xi;
            double eta_d = eta;
            double sigma_d = 1;
            double tau_d = 0;
            for (int i = 1; i <= 5; i++)
            {
                int i2 = 2 * i;
                xi_d -= beta[i] * Math.Sin(i2 * xi) * Math.Cosh(i2 * eta);
                eta_d -= beta[i] * Math.Cos(i2 * xi) * Math.Sinh(i2 * eta);
                sigma_d -= beta[i] * Math.Cos(i2 * xi) * Math.Cosh(i2 * eta) * i2;
                tau_d += beta[i] * Math.Sin(i2 * xi) * Math.Sinh(i2 * eta) * i2;
            }

            double chi = Math.Asin(Math.Sin(xi_d) / Math.Cosh(eta_d));

            double latitude = chi;
            for (int i = 1; i <= 6; i++)
            {
                latitude += delta[i] * Math.Sin(2 * i * chi);
            }

            return new GeoCoordinate(
                Rad2Deg(latitude),
                Rad2Deg(longitude0 + Math.Atan(Math.Sinh(eta_d) / Math.Cos(xi_d))),
                cartesian.y - origin.Altitude
            );
        }

        private static double Deg2Rad(double deg) => deg * Math.PI / 180d;
        private static double Rad2Deg(double rad) => rad * 180d / Math.PI;
    }
}
