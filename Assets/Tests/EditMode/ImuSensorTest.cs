using NUnit.Framework;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.TestTools.Utils;
using AWSIM;
using AWSIM.Tests.Utilities;

public class ImuSensorTest
{
    FloatEqualityComparer floatEqualityComparer;
    Vector3EqualityComparer vector3EqualityComparer;

    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        floatEqualityComparer = new FloatEqualityComparer(10e-6f);
        vector3EqualityComparer = new Vector3EqualityComparer(10e-6f);
    }

    [Test]
    public void EnableGravity()
    {
        bool dataReceived = false;
        GameObject go = new GameObject();
        var imuSensor = go.AddComponent<ImuSensor>();
        imuSensor.EnableGravity = true;

        imuSensor.OnOutputData += (ImuSensor.OutputData data) =>
        {
            dataReceived = true;
            Assert.That(data.LinearAcceleration.y, Is.EqualTo(Physics.gravity.y).Using(floatEqualityComparer));
        };

        imuSensor.Call("Start");
        imuSensor.SetPrivateFieldValue<float>("timer", 1.0f);
        imuSensor.Call("FixedUpdate");

        Assert.IsTrue(dataReceived);
        UnityEngine.Object.DestroyImmediate(go);
    }

    [Test]
    public void DisableGravity()
    {
        bool dataReceived = false;
        GameObject go = new GameObject();
        var sensor = go.AddComponent<ImuSensor>();
        sensor.EnableGravity = false;

        sensor.OnOutputData += (ImuSensor.OutputData data) =>
        {
            dataReceived = true;
            Assert.That(data.LinearAcceleration, Is.EqualTo(Vector3.zero).Using(vector3EqualityComparer));
        };

        sensor.Call("Start");
        sensor.SetPrivateFieldValue<float>("timer", 1.0f);
        sensor.Call("FixedUpdate");

        Assert.IsTrue(dataReceived);
        UnityEngine.Object.DestroyImmediate(go);
    }
}
