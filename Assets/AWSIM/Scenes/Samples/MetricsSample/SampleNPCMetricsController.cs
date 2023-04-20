using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.Samples {
    public class SampleNPCMetricsController : MonoBehaviour
    {
        [SerializeField] NPCVehicle npcVehicle;
        Vector3 currentPosition;
        Quaternion currentRotation;
        // Start is called before the first frame update
        void Start()
        {
            StartCoroutine(Routine());
        }

        IEnumerator Routine()
        {
            yield return UpdatePosAndRot(5f, 3f, 0f);
        }

        IEnumerator UpdatePosAndRot(float duration, float speed, float yawSpeed, bool validatePose = true)
        {
            var startTime = Time.fixedTime;
            yield return new WaitForFixedUpdate();
            while (Time.fixedTime - startTime < duration)
            {
                var euler = currentRotation.eulerAngles;
                currentRotation = Quaternion.Euler(euler.x, euler.y + yawSpeed * Time.fixedDeltaTime, euler.z);
                currentPosition += currentRotation * Vector3.forward * speed * Time.fixedDeltaTime;
                npcVehicle.SetRotation(currentRotation);
                npcVehicle.SetPosition(currentPosition);
                yield return new WaitForFixedUpdate();
            }
        }
    }
}
