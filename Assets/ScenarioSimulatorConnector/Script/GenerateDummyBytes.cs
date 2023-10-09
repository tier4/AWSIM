using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SimulationApiSchema;
using System;
using Google.Protobuf;

public class GenerateDummyBytes : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        DebugInitializeRequest();
    }

    void DebugInitializeRequest()
    {
        var request = new InitializeRequest
        {
            RealtimeFactor = 1,
            StepTime = 0.033,
            // TODO: initialize time
            // TODO: initialzie ros time 
        };

        var bytes = request.ToByteArray();
        CsharpToPython(bytes);
    }

    void DebugUpdateFrameRequest()
    {
        var request = new UpdateFrameRequest
        {
            CurrentSimulationTime = 2,
            CurrentRosTime = new BuiltinInterfaces.Time(),
        };

        var bytes = request.ToByteArray();
        CsharpToPython(bytes);
    }

    void DebugSpawnVehicleEntityRequest()
    {
        var request = new SpawnVehicleEntityRequest
        {
            Parameters = new TrafficSimulatorMsgs.VehicleParameters
            {
                Name = "id_1",
            },
            IsEgo = false,
            // TODO: asset key
            // TODO: pose
        };

        var bytes = request.ToByteArray();
        CsharpToPython(bytes);
    }

    void DebugDespawnEntityRequest()
    {
        var request = new DespawnEntityRequest
        {
            Name = "id_1",
        };

        var bytes = request.ToByteArray();
        CsharpToPython(bytes);
    }

    static void CsharpToPython(byte[] bytes)
    {
        var s = "-" + BitConverter.ToString(bytes);
        Debug.Log("initialize_request : " + s.Replace("-", "\\x"));
    }

    // Update is called once per frame
    void Update()
    {

    }
}
