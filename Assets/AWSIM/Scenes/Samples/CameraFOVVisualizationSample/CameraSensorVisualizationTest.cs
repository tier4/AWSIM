using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace AWSIM.Tests
{
    /// <summary>
    ///  Class for testing CameraSensorVisualization
    /// </summary>
    public class CameraSensorVisualizationTest : MonoBehaviour
    {

        [Header("Components")]
        [SerializeField] private Camera cameraSensor = default;

        [SerializeField] private Transform dummyCar = default;

        [SerializeField] private CameraSensorVisualization cameraSensorVisualization = default;

        [Header("Sensors")]
        [SerializeField] private GameObject leftSensor = default;
        [SerializeField] private GameObject rightSensor = default;

        [Header("Variables")]
        [SerializeField] private AnimationCurve swapPath = default;


        private Vector3[] meshVerticesFov60 = new Vector3[] {
            new Vector3(-0.3f, -0.16875f, 0.3f),
            new Vector3(-0.3f, 0.16875f, 0.3f),
            new Vector3(0.3f, 0.16875f, 0.3f),
            new Vector3(0.3f, -0.16875f, 0.3f),
            new Vector3(-100f, -56.25f, 100f),
            new Vector3(-100f, 56.25f, 100f),
            new Vector3(100f, 56.25f, 100f),
            new Vector3(100f, -56.25f, 100f),
        };

        private Vector3[] meshVerticesFov89 = new Vector3[] {
            new Vector3(-0.525f, -0.2953124f, 0.3f),
            new Vector3(-0.525f, 0.2953124f, 0.3f),
            new Vector3(0.525f, 0.2953124f, 0.3f),
            new Vector3(0.525f, -0.2953124f, 0.3f),
            new Vector3(-175f, -98.43747f, 100f),
            new Vector3(-175f, 98.43747f, 100f),
            new Vector3(175f, 98.43747f, 100f),
            new Vector3(175f, -98.43747f, 100f),
        };


        public void StartTest()
        {
            StartCoroutine(TestRoutine());
        }

        private IEnumerator TestRoutine()
        {
            yield return StartCoroutine(MeshInitializationTestRoutine());
            yield return StartCoroutine(MeshFor60FovTestRoutine());
            yield return StartCoroutine(MeshFor89FovTestRoutine());
            yield return StartCoroutine(SwapCameraFovTestRoutine());
            yield return StartCoroutine(MoveAndRotateVisualizerTestRoutine());       
            yield return StartCoroutine(ThreeSensorTestRoutine());
            yield return StartCoroutine(ResetTestRoutine());
        }

        private IEnumerator MeshInitializationTestRoutine()
        {
            yield return null;

            if(cameraSensorVisualization.FillMesh == null)
            {
                LogResult(false, "Fill Mesh has not been initialized.");
            }
            else
            {
                LogResult(true, "Fill Mesh initialized.");
            }

            if(cameraSensorVisualization.EdgeMesh == null)
            {
                LogResult(false, "Edge Mesh has not been initialized.");
            }
            else
            {
                LogResult(true, "Edge Mesh initialized.");
            }

            if(cameraSensorVisualization.DepthMesh == null)
            {
                LogResult(false, "Depth Mesh has not been initialized.");
            }
            else
            {
                LogResult(true, "Depth Mesh initialized.");
            }

        }

        private IEnumerator MeshFor60FovTestRoutine()
        {
            yield return null;

            // set up //    
            cameraSensorVisualization.transform.localPosition = Vector3.zero;
            cameraSensorVisualization.transform.localRotation = Quaternion.identity;

            cameraSensorVisualization.gameObject.SetActive(false);
            yield return null;

            cameraSensor.nearClipPlane = 0.3f;
            cameraSensor.farClipPlane = 100f;
            cameraSensor.sensorSize = new Vector2(70.0f, 39.4f);
            cameraSensor.focalLength = 35f;

            cameraSensorVisualization.gameObject.SetActive(true);

            // mesh vertices are created on update, so just after enable number of vertices are zero
            for (int i = 0; i < 5; i++)
            {
                yield return new WaitForSeconds(0.1f);

                if(cameraSensorVisualization.FillMesh.vertices.Length > 0)
                {
                    break;
                }
            }

            if(cameraSensorVisualization.FillMesh.vertices.Length != meshVerticesFov60.Length)
            {
                LogResult(false, "Number of vertices is not correct.");
                yield break;
            }

            for(int i=0; i<cameraSensorVisualization.FillMesh.vertices.Length; i++)
            {
                if(!IsEqual(cameraSensorVisualization.FillMesh.vertices[i],
                    meshVerticesFov60[i], 0.001f))
                {

                    LogResult(false, "Mesh for FOV 60 is not correct.");
                    yield break;
                } 
            }

            LogResult(true, "Mesh for FOV 60 is correct.");
        }

        private IEnumerator MeshFor89FovTestRoutine()
        {
            yield return null;  

            // set up //
            cameraSensorVisualization.transform.localPosition = Vector3.zero;
            cameraSensorVisualization.transform.localRotation = Quaternion.identity;

            cameraSensorVisualization.gameObject.SetActive(false);
            yield return null;

            cameraSensor.nearClipPlane = 0.3f;
            cameraSensor.farClipPlane = 100f;
            cameraSensor.sensorSize = new Vector2(70.0f, 39.4f);
            cameraSensor.focalLength = 20f;

            cameraSensorVisualization.gameObject.SetActive(true);     

            // mesh vertices are created on update, so just after enable number of vertices are zero
            for (int i = 0; i < 5; i++)
            {
                yield return new WaitForSeconds(0.1f);

                if(cameraSensorVisualization.FillMesh.vertices.Length > 0)
                {
                    break;
                }
            }

            if(cameraSensorVisualization.FillMesh.vertices.Length != meshVerticesFov89.Length)
            {
                LogResult(false, "Number of vertices is not correct.");
                yield break;
            }
            
            for(int i=0; i<cameraSensorVisualization.FillMesh.vertices.Length; i++)
            {
                if(!IsEqual(cameraSensorVisualization.FillMesh.vertices[i],
                    meshVerticesFov89[i], 0.001f))
                {
                    LogResult(false, "Mesh for FOV 89 is not correct.");
                    yield break;
                } 
            }

            LogResult(true, "Mesh for FOV 89 is correct.");

        }

        private IEnumerator SwapCameraFovTestRoutine()
        {
            LogMessage("Start Camera Fov Swap ...");

            float minFov = 1f;
            float maxFov = 179f;

            float time = 0f;
            float value = 0f;
            float duration = 4f;

            while (time < duration)
            {                
                value = minFov + swapPath.Evaluate(time / duration) * (maxFov - minFov);
                cameraSensor.fieldOfView = value;

                time += Time.deltaTime;
                yield return null;
            }

            time = 0f;
            while (time < duration)
            {                
                value = minFov + (1f - swapPath.Evaluate(time / duration)) * (maxFov - minFov);
                cameraSensor.fieldOfView = value;

                time += Time.deltaTime;
                yield return null;
            }

            LogMessage("Camera Fov Swap Completed", 0);
        }

        private IEnumerator MoveAndRotateVisualizerTestRoutine()
        {
            LogMessage("Start Camera Fov Movement ...");

            if(cameraSensor.transform.parent == null)
            {
                LogMessage("For testing purpose please attach camera to ForwardSensor gameobject.", 1);
                yield break;
            }

            if(cameraSensor.transform.parent.parent != dummyCar)
            {
                LogMessage("For testing purpose please attach camera and ForwardSensor to DummyCar gameobject.", 1);
                yield break;
            }

            cameraSensor.sensorSize = new Vector2(70.0f, 39.4f);
            cameraSensor.focalLength = 35f;

            float duration = 4f;
            float speed = 2f;
            
            float time = 0f;
            Vector3 direction = new Vector3(0.3f, 0.1f, 0.7f).normalized;

            Quaternion startQuaternion = dummyCar.rotation;
            Quaternion targetQuaternion = Quaternion.Euler(-17f, 36f, 0f);

            while(time < duration)
            {
                time += Time.deltaTime;

                dummyCar.position += direction * speed * Time.deltaTime;
                dummyCar.rotation = Quaternion.Lerp(startQuaternion, targetQuaternion, time / duration); 

                yield return null;
            }

            time = 0f;
            direction = new Vector3(-0.4f, 0.0f, 0.4f).normalized;

            startQuaternion = dummyCar.rotation;
            targetQuaternion = Quaternion.Euler(13f, -74f, 3f);

            while(time < duration)
            {
                time += Time.deltaTime;

                dummyCar.position += direction * speed * Time.deltaTime;
                dummyCar.rotation = Quaternion.Lerp(startQuaternion, targetQuaternion, time / duration); 

                yield return null;
            }


            dummyCar.position = new Vector3(0f, 1f, 0f);
            dummyCar.rotation = Quaternion.identity;

            LogMessage("Camera Fov Movement Completed", 0);    

        }

        private IEnumerator ThreeSensorTestRoutine()
        {
            leftSensor.SetActive(true);
            rightSensor.SetActive(true);

            yield return new WaitForSeconds(0.4f);

            yield return MoveAndRotateVisualizerTestRoutine();

            yield return new WaitForSeconds(0.4f);

            leftSensor.SetActive(false);
            rightSensor.SetActive(false);
        }

        private IEnumerator ResetTestRoutine()
        {
            cameraSensor.fieldOfView = 60f;
            cameraSensor.nearClipPlane = 0.3f;
            cameraSensor.sensorSize = new Vector2(70.0f, 39.4f);
            cameraSensor.focalLength = 35f;

            yield return null;
        }


        private void LogResult(bool success, string message) 
        {
            if(success)
            {
                Debug.Log("<color=green>TEST SUCCESS </color>" + message);
            }
            else
            {
                Debug.Log("<color=red>TEST FAILED </color>" + message);
            }            
        }

        private void LogMessage(string message, int colorId = -1)
        {
            if(colorId == 0)
            {
                Debug.Log("<color=green>TEST </color>" + message);
            }
            else if(colorId == 1)
            {
                Debug.Log("<color=red>TEST </color>" + message);
            }
            else
            {
                Debug.Log("<color=yellow>TEST </color>" + message);
            }
        }

        private bool IsEqual(Vector3 a, Vector3 b, float precision)
        {        
            if (Vector3.Magnitude(a - b) < precision)
            {
                return true;
            }
            return false;
        }
    }

#if UNITY_EDITOR

    [CustomEditor(typeof(CameraSensorVisualizationTest), true)]
    public class CameraSensorVisualizationTest_Inspector : Editor 
    {

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            CameraSensorVisualizationTest test = (CameraSensorVisualizationTest) target;

            GUILayout.Space(20f);
            if (GUILayout.Button("TEST"))
            {
                test.StartTest();
            }
            
            GUILayout.Space(5f);
            EditorGUILayout.HelpBox("To test features the editor has to be in PLAY mode.", MessageType.Info);

        }

    }

#endif

}