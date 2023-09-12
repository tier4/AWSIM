using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.PhysicsTest
{
    public class WheelBehaviour : MonoBehaviour
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

            if (_currTime >= 4f && _currTime < 14f)
            {
                speed = 1.5f;
            }
            else if (_currTime >= 14f && _currTime < 24f)
            {
                speed = 0f;
            }
            else if (_currTime >= 24f && _currTime < 34f)
            {
                speed = -1.5f;
            }
            else
            {
                speed = 0f;
            }
            _currTime += Time.fixedDeltaTime;

            Vector3 direction = (this.transform.rotation * new Vector3(1f, 0f, 0f)).normalized;
            Vector3 forwardDirection = ((Quaternion.AxisAngle(Vector3.up, -90f * Mathf.Deg2Rad)) * direction).normalized;
            Vector3 impulse = forwardDirection * Time.fixedDeltaTime * speed;

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
