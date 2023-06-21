using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.PhysicsTest
{
    public class BallBehaviour : MonoBehaviour
    {
        [SerializeField] private Rigidbody _rigidBody = default;

        private OutputJson _output = default;
        private float _currTime = 0f;


        private void OnEnable()
        {
            _currTime = 0f;
            _output = new OutputJson();
        }

        private void OnDisable() 
        {
            string outputPath = SaveData();
            Debug.Log("DONE " + outputPath);
        }

        private void FixedUpdate()
        {
            float fixedDeltaTime = Time.fixedDeltaTime;
            float speed = 0f;

            if (_currTime >= 2f && _currTime < 7f)
            {
                speed = 1.5f;
            }
            else if (_currTime >= 7f && _currTime < 12f)
            {
                speed = 0f;
            }
            else if (_currTime >= 12f && _currTime < 17f)
            {
                speed = -1.5f;
            }
            else
            {
                speed = 0f;
            }

            _currTime += Time.fixedDeltaTime;

            Vector3 direction = Vector3.forward;
            Vector3 impulse = direction * Time.fixedDeltaTime * speed;

            _rigidBody.AddForce(impulse, ForceMode.Impulse);

            OutputData data = new()
            {
                Time = _currTime,
                PosX = this.transform.position.x,
                PosY = this.transform.position.y,
                PosZ = this.transform.position.z,
                VelX = _rigidBody.velocity.x,
                VelY = _rigidBody.velocity.y,
                VelZ = _rigidBody.velocity.z,
            };

            _output.Data.Add(data);
        }

        private string SaveData()
        {
            string jsonData = JsonUtility.ToJson(_output);
            string jsonPath = Application.persistentDataPath + "/oop_scenario_.json";
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
