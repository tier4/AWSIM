using NUnit.Framework;
using UnityEngine;
using AWSIM;
using UnityEngine.TestTools.Utils;

namespace Sensors
{
    public class IMU
    {
        // Comparers
        FloatEqualityComparer floatComparer;
        Vector3EqualityComparer v3Comparer;
        
        [OneTimeSetUp]
        public void OneTimeSetUp() {
            floatComparer = new FloatEqualityComparer(10e-6f);
            v3Comparer = new Vector3EqualityComparer(10e-6f);
        }

        [Test]
        public void Gravity()
        {
            bool dataReceived = false;
            GameObject go = new GameObject();
            var sensor =  go.AddComponent<ImuSensor>();
            sensor.EnableGravity = true;

            sensor.OnOutputData += (ImuSensor.OutputData data) => {
                dataReceived = true;
                Assert.That(data.LinearAcceleration.y, Is.EqualTo(Physics.gravity.y).Using(floatComparer));
            };

            sensor.call("Start");
            sensor.SetPrivateFieldValue<float>("timer", 1.0f);
            sensor.call("FixedUpdate");

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }

        [Test]
        public void NoGravity()
        {
            bool dataReceived = false;
            GameObject go = new GameObject();
            var sensor =  go.AddComponent<ImuSensor>();
            sensor.EnableGravity = false;

            sensor.OnOutputData += (ImuSensor.OutputData data) => {
                dataReceived = true;
                Assert.That(data.LinearAcceleration, Is.EqualTo(Vector3.zero).Using(v3Comparer));
            };

            sensor.call("Start");
            sensor.SetPrivateFieldValue<float>("timer", 1.0f);
            sensor.call("FixedUpdate");

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }
    }

    public class GNSS
    {
        FloatEqualityComparer floatComparer;
        Vector3EqualityComparer v3Comparer;
        
        [OneTimeSetUp]
        public void OneTimeSetUp() {
            floatComparer = new FloatEqualityComparer(10e-6f);
            v3Comparer = new Vector3EqualityComparer(10e-6f);
        }

        [Test]
        public void MgrsZero()
        {
            bool dataReceived = false;
            GameObject go = new GameObject();
            var sensor =  go.AddComponent<GnssSensor>();
            sensor.OutputHz = 1;
            go.AddComponent<Environment>();

            sensor.OnOutputData += (GnssSensor.OutputData data) => {
                dataReceived = true;
                Assert.That(data.MgrsPosition, Is.EqualTo(Vector3.zero).Using(v3Comparer));
            };

            sensor.call("Start");
            sensor.SetPrivateFieldValue<float>("timer", 1.0f);
            sensor.call("FixedUpdate");

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }

        static Vector3[] gnssMgrsNonZeroValues = new Vector3[] { 
            Vector3.one,
            new Vector3(-1.0f, 2.0f, 0.0f),
            Vector3.up
         };

        [Test]
        public void MgrsNonZero([ValueSource("gnssMgrsNonZeroValues")] Vector3 value)
        {
            bool dataReceived = false;
            GameObject go = new GameObject();
            var sensor =  go.AddComponent<GnssSensor>();
            var env = go.AddComponent<Environment>();
            env.SetPrivateFieldValue<Vector3>("mgrsOffsetPosition", value);

            sensor.OnOutputData += (GnssSensor.OutputData data) => {
                dataReceived = true;
                Assert.That(data.MgrsPosition, Is.EqualTo(value).Using(v3Comparer));
            };

            sensor.call("Start");
            sensor.SetPrivateFieldValue<float>("timer", 1.0f);
            sensor.call("FixedUpdate");

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }

        public static object[] gnssTranslateCases =
        {
            new object[] { new Vector3(1.0f, 0.0f, 1.0f), new Vector3(1.0f, -1.0f, 0.0f) },
            new object[] { new Vector3(-1.0f, 0.0f, 0.0f), new Vector3(0.0f, 1.0f, 0.0f) },
            new object[] { new Vector3(-1.0f, -1.0f, -1.0f), new Vector3(-1.0f, 1.0f, -1.0f) },
        };
        [TestCaseSource(nameof(gnssTranslateCases))]
        public void Translate(Vector3 translate, Vector3 expectedResult)
        {
            bool dataReceived = false;
            GameObject go = new GameObject();
            var sensor =  go.AddComponent<GnssSensor>();
            sensor.OutputHz = 1;
            var env = go.AddComponent<Environment>();
            
            go.transform.Translate(translate);

            sensor.OnOutputData += (GnssSensor.OutputData data) => {
                dataReceived = true;
                Assert.That(data.MgrsPosition, Is.EqualTo(expectedResult).Using(v3Comparer));
            };

            sensor.call("Start");
            sensor.SetPrivateFieldValue<float>("timer", 1.0f);
            sensor.call("FixedUpdate");

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }
    }
}

