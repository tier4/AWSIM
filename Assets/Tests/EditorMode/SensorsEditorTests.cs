using NUnit.Framework;
using UnityEngine;
using AWSIM;

namespace Sensors
{
    public class IMU
    {
        [Test]
        public void Gravity()
        {
            bool dataReceived = false;
            GameObject go = new GameObject();
            var sensor =  go.AddComponent<ImuSensor>();
            sensor.EnableGravity = true;

            sensor.OnOutputData += (ImuSensor.OutputData data) => {
                dataReceived = true;
                Assert.AreEqual(data.LinearAcceleration.y, Physics.gravity.y);
            };

            sensor.call("Start");
            for(int i=0; i<10; i++)
            {
                sensor.call("FixedUpdate");
            }

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
                Assert.AreEqual(data.LinearAcceleration, Vector3.zero);
            };

            sensor.call("Start");
            for(int i=0; i<10; i++)
            {
                sensor.call("FixedUpdate");
            }

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }
    }

    public class GNSS
    {
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
                Assert.AreEqual(data.MgrsPosition, Vector3.zero);
            };

            sensor.call("Start");
            for(int i=0; i<10; i++)
            {
                sensor.call("FixedUpdate");
            }

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }

        [Test]
        public void MgrsNonZero()
        {
            bool dataReceived = false;
            GameObject go = new GameObject();
            var sensor =  go.AddComponent<GnssSensor>();
            var env = go.AddComponent<Environment>();
            env.SetPrivateFieldValue<Vector3>("mgrsOffsetPosition", Vector3.one);

            sensor.OnOutputData += (GnssSensor.OutputData data) => {
                dataReceived = true;
                Assert.AreEqual(data.MgrsPosition, Vector3.one);
            };

            sensor.call("Start");
            for(int i=0; i<10; i++)
            {
                sensor.call("FixedUpdate");
            }

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }

        [Test]
        public void Translate()
        {
            bool dataReceived = false;
            GameObject go = new GameObject();
            var sensor =  go.AddComponent<GnssSensor>();
            sensor.OutputHz = 1;
            var env = go.AddComponent<Environment>();
            
            go.transform.Translate(new Vector3(1.0f, 0.0f, 1.0f));

            sensor.OnOutputData += (GnssSensor.OutputData data) => {
                dataReceived = true;
                Assert.AreEqual(data.MgrsPosition, new Vector3(1.0f, -1.0f, 0.0f));
            };

            sensor.call("Start");
            for(int i=0; i<10; i++)
            {
                sensor.call("FixedUpdate");
            }

            Assert.IsTrue(dataReceived);
            UnityEngine.Object.DestroyImmediate(go);
        }
    }
}

