using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;

namespace AWSIM.PhysicsTest
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    partial class BallDataSaveSystem : SystemBase
    {
        private OutputJson _output = default;
        private float _currTime = 0f;

        protected override void OnStartRunning()
        {
            _currTime = 0f;
            _output = new OutputJson();
        }

        protected override void OnStopRunning()
        {
            string outputPath = SaveData();
            Debug.Log("DONE " + outputPath);
        }

        protected override void OnUpdate()
        {
            float deltaTime = Time.DeltaTime;

            Entities
                .WithAll<BallTag>()
                .WithoutBurst()
                .ForEach((in Translation translation, in PhysicsVelocity velocity) =>
                {
                    OutputData data = new()
                    {
                        Time = _currTime,
                        PosX = translation.Value.x,
                        PosY = translation.Value.y,
                        PosZ = translation.Value.z,
                        VelX = velocity.Linear.x,
                        VelY = velocity.Linear.y,
                        VelZ = velocity.Linear.z,
                    };

                    _output.Data.Add(data);
                    _currTime += deltaTime;
                }).Run();
        }

        private string SaveData()
        {
            string jsonData = JsonUtility.ToJson(_output);
            string jsonPath = Application.persistentDataPath + "/dots_scenario_.json";
            System.IO.File.WriteAllText(jsonPath, jsonData);
            return jsonPath;
        }

        [System.Serializable]
        public class OutputJson
        {
            public List<OutputData> Data = new List<OutputData>();
        }

        [System.Serializable]
        public class OutputData
        {
            public float Time;

            public float PosX;
            public float PosY;
            public float PosZ;

            public float VelX;
            public float VelY;
            public float VelZ;
        }
    }
}