using NUnit.Framework;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.TestTools.Utils;
using AWSIM;

public class ImuSensorTest
{
    FloatEqualityComparer floatComparer;
    Vector3EqualityComparer v3Comparer;

    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        floatComparer = new FloatEqualityComparer(10e-6f);
        v3Comparer = new Vector3EqualityComparer(10e-6f);
    }

    [Test]
    public void Gravity()
    {
        bool dataReceived = false;
        GameObject go = new GameObject();
        var sensor = go.AddComponent<ImuSensor>();
        sensor.EnableGravity = true;

        sensor.OnOutputData += (ImuSensor.OutputData data) => {
            dataReceived = true;
            Assert.That(data.LinearAcceleration.y, Is.EqualTo(Physics.gravity.y).Using(floatComparer));
        };

        sensor.Call("Start");
        sensor.SetPrivateFieldValue<float>("timer", 1.0f);
        sensor.Call("FixedUpdate");

        Assert.IsTrue(dataReceived);
        UnityEngine.Object.DestroyImmediate(go);
    }

    [Test]
    public void NoGravity()
    {
        bool dataReceived = false;
        GameObject go = new GameObject();
        var sensor = go.AddComponent<ImuSensor>();
        sensor.EnableGravity = false;

        sensor.OnOutputData += (ImuSensor.OutputData data) => {
            dataReceived = true;
            Assert.That(data.LinearAcceleration, Is.EqualTo(Vector3.zero).Using(v3Comparer));
        };

        sensor.Call("Start");
        sensor.SetPrivateFieldValue<float>("timer", 1.0f);
        sensor.Call("FixedUpdate");

        Assert.IsTrue(dataReceived);
        UnityEngine.Object.DestroyImmediate(go);
    }
}
