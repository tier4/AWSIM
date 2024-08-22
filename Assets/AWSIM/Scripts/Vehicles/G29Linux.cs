using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace AWSIM
{
    public static class G29Linux
    {
        [DllImport("libG29Linux")]
        public static extern bool InitDevice(string deviceName);

        [DllImport("libG29Linux")]
        public static extern void UploadEffect(double toruqe, double attack_length);

        [DllImport("libG29Linux")]
        public static extern double GetPos();
    }
}