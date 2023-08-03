using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// CameraSensorHolder.
    /// Controls the rendering sequence of multiple camera sensors.
    /// </summary>
    public class CameraSensorHolder : MonoBehaviour
    {
        [Header("Camera Sensors")]
        [SerializeField] private List<CameraSensor> cameraSensors = default;

        [Header("Parameters")]

        /// <summary>
        /// Data output hz.
        /// Sensor processing and callbacks are called in this hz.
        /// </summary>
        [Range(0, 30)][SerializeField] private uint publishHz = 10;

        /// <summary>
        /// Rendering sequence type.
        /// Set True for sensors render at different frames one after another.
        /// Set False for all sensors render at the same frame.
        /// </summary>
        [SerializeField] private bool renderInQueue = true;

        float timer = 0;

        private void Awake() 
        {
            if(cameraSensors == null || cameraSensors.Count < 1)
            {
                Debug.LogError("Camera sensor list should have at least one camera to render.");
                return;
            }

            StartCoroutine(FixedUpdateRoutine());
        }

        private IEnumerator FixedUpdateRoutine()
        {
            timer = 0f;

            while(true)
            {
                yield return new WaitForFixedUpdate();

                // Update timer.
                timer += Time.deltaTime;

                // Matching output to hz.
                var interval = 1.0f / (int)publishHz;
                if (timer < interval)
                {
                    continue;
                }
                timer = 0f;

                // sensors render at different frames one after another
                if(renderInQueue)
                {
                    for (int i = 0; i < cameraSensors.Count; i++)
                    {
                        yield return StartCoroutine(RenderCamera(cameraSensors[i], true));
                    }
                }
                // sensors render at the same frame
                else
                {
                    for (int i = 0; i < cameraSensors.Count; i++)
                    {
                        StartCoroutine(RenderCamera(cameraSensors[i], false));
                    }
                }

                yield return new WaitForFixedUpdate();
            }
        }

    /// <summary>
    /// Call camera sensor to do a render.
    /// </summary>
    /// <param name="cameraSensor">Camera sensor to render.</param>
    /// <param name="wait">Set True if wait to end of the frame after render.</param>
        private IEnumerator RenderCamera(CameraSensor cameraSensor, bool wait) 
        {
            if(cameraSensor.gameObject.activeInHierarchy)
            {
                cameraSensor.DoRender();
            }

            if(wait)
            {
                yield return new WaitForEndOfFrame();
            }
            else
            {
                yield return null;
            }
        }
    }
}
