using NUnit.Framework;
using AWSIM;
using AWSIM.TrafficSimulation;
using UnityEngine.TestTools.Utils;

using UnityEngine;

namespace Traffic
{
    public class TrafficPrimitives
    {
        Vector3EqualityComparer v3Comparer;
        
        [OneTimeSetUp]
        public void OneTimeSetUp() {
            // floatComparer = new FloatEqualityComparer(10e-6f);
            v3Comparer = new Vector3EqualityComparer(10e-6f);
        }
        
        [Test]
        public void StopLineCentrerPoint()
        {
            StopLine stopLine = StopLine.Create(Vector3.zero, new Vector3(1, 0, 0));
            Assert.That(stopLine.CenterPoint, Is.EqualTo(new Vector3(0.5f, 0, 0)).Using(v3Comparer));
        }


        static Vector3[][] trafficLaneWaypoints = new Vector3[][] { 
            new Vector3[] {new Vector3(0,0,0), new Vector3(1,0,0), new Vector3(2,0,0) },
            new Vector3[] {new Vector3(-1,0,1), new Vector3(1,0,1), new Vector3(2,0,2) },
            new Vector3[] {new Vector3(0,0,1), new Vector3(0,0,2), new Vector3(0,0,1) },
            new Vector3[] {new Vector3(-10,0,10), new Vector3(0,0,0), new Vector3(10,0,-10) },
        };
        [Test]
        public void TrafficLanePositions([ValueSource("trafficLaneWaypoints")] Vector3[] waypoints)
        {
            TrafficLane trafficLane = TrafficLane.Create(waypoints, TrafficLane.TurnDirectionType.STRAIGHT);
            Assert.That(trafficLane.transform.position, Is.EqualTo(waypoints[0]));
        }

        [Test]
        public void TrafficLaneWithStopLine()
        {
            var waypoints = new Vector3[] {new Vector3(0, 0, -1.0f), new Vector3(0, 0, 0), new Vector3(0, 0, 1.0f) };
            TrafficLane trafficLane = TrafficLane.Create(waypoints, TrafficLane.TurnDirectionType.STRAIGHT);
            StopLine stopLine = StopLine.Create(new Vector3(-0.5f, 0, 0.2f), new Vector3(0.5f, 0, 0.2f));
            trafficLane.StopLine = stopLine;
            Assert.That(trafficLane.GetStopPoint(), Is.EqualTo(new Vector3(0.0f, 0, 0.2f)).Using(v3Comparer));
        }

        [Test]
        public void TrafficLaneWithoutStopLine()
        {
            var waypoints = new Vector3[] {new Vector3(0, 0, -1.0f), new Vector3(0, 0, 0), new Vector3(0, 0, 1.0f) };
            TrafficLane trafficLane = TrafficLane.Create(waypoints, TrafficLane.TurnDirectionType.STRAIGHT);
            Assert.That(trafficLane.GetStopPoint(), Is.EqualTo(waypoints[0]).Using(v3Comparer));
        }
    }
}

