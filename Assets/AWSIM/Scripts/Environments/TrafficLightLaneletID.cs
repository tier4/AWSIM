using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Timers;

namespace AWSIM
{
    [RequireComponent(typeof(TrafficLight))]
    public class TrafficLightLaneletID : MonoBehaviour
    {
        public long LaneletElementID = 0;
    }
}
