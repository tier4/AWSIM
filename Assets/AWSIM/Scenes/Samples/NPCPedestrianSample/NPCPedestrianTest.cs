using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.Tests
{
    public class NPCPedestrianTest : MonoBehaviour
    {
        [SerializeField] private GameObject npcPedestrianPrefab;
        [SerializeField] private Transform spawnPoint;

        private NPCPedestrian npc;
        private Vector3 currentPosition;
        private Quaternion currentRotation;

        private void Start()
        {
            StartCoroutine(TestRoutine());
        }

        private IEnumerator TestRoutine()
        {
            npc = Instantiate(npcPedestrianPrefab).GetComponent<NPCPedestrian>();
            currentPosition = spawnPoint.position;
            npc.SetPosition(currentPosition);
            currentRotation = spawnPoint.rotation;
            npc.SetRotation(currentRotation);
            Debug.Log("NPC Pedestrian Walk");
            yield return TestMoveForwardRoutine(3f, 1f);
            Debug.Log("NPC Pedestrian Run");
            yield return TestMoveForwardRoutine(3f, 2f);
            Debug.Log("NPC Pedestrian Run Faster");
            yield return TestMoveForwardRoutine(3f, 3f);
            Debug.Log("NPC Pedestrian Idle");
            yield return TestStandRoutine(3f);
            Debug.Log("NPC Pedestrian Rotate Still");
            yield return TestRotateStillRoutine(3f, 30f);
            Debug.Log("NPC Pedestrian Walk Step");
            yield return TestMoveForwardRoutine(3f, 1f);
            Time.timeScale = 0.2f;
            Debug.Log($"NPC Pedestrian Slow Motion Run(Time Scale = {Time.timeScale})");
            yield return TestMoveForwardRoutine(3f, 3f);

        }

        private IEnumerator TestMoveForwardRoutine(float duration, float speed)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                yield return new WaitForFixedUpdate();
                currentPosition += currentRotation * Vector3.forward * speed * Time.fixedDeltaTime;
                npc.SetPosition(currentPosition);
            }
        }

        private IEnumerator TestStandRoutine(float duration)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                yield return new WaitForFixedUpdate();
                // Do nothing
            }
        }

        private IEnumerator TestRotateStillRoutine(float duration, float angularSpeed)
        {
            var startTime = Time.fixedTime;
            while (Time.fixedTime - startTime < duration)
            {
                yield return new WaitForFixedUpdate();
                currentRotation *= Quaternion.AngleAxis(angularSpeed * Time.fixedDeltaTime, Vector3.up);
                npc.SetRotation(currentRotation);
            }
        }
    }
}
