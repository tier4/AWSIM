using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

namespace AWSIM.Samples
{

    public class SampleNPCVehicleController : MonoBehaviour
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

            Debug.Log("--- Start control NPCVehicle ---");
            Debug.Log("Straight");
            yield return UpdatePosAndRot(5f, 3f, 0f);
            Debug.Log("Turn right");
            yield return UpdatePosAndRot(13.4f, 3f, 20f);
            Debug.Log("Straight");
            yield return UpdatePosAndRot(2f, 3f, 0f);
            Debug.Log("Left turn signal");
            npcVehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.LEFT);
            yield return new WaitForSeconds(2f);
            Debug.Log("Right turn signal");
            npcVehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.RIGHT);
            yield return new WaitForSeconds(2f);
            Debug.Log("Hazard signal");
            npcVehicle.SetTurnSignalState(NPCVehicle.TurnSignalState.HAZARD);
            Debug.Log("Back Right");
            yield return UpdatePosAndRot(3f, -2f, -30f);
            Debug.Log("Back");
            yield return UpdatePosAndRot(2f, -2f, 0f);
            Debug.Log("--- End control NPC Vehicle ---");
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
                if (validatePose)
                {
                    Assert.AreApproximatelyEqual(currentPosition.x, npcVehicle.transform.position.x, 0.1f);
                    Assert.AreApproximatelyEqual(currentPosition.z, npcVehicle.transform.position.z, 0.1f);
                    Assert.AreApproximatelyEqual(currentRotation.eulerAngles.y,
                        npcVehicle.transform.rotation.eulerAngles.y, 0.1f);
                }
            }
        }
    }
}