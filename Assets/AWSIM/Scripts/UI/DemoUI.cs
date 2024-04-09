using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    public class DemoUI : MonoBehaviour
    {
        [SerializeField] Text versionText;
        [SerializeField] Text fpsText;

        int frameCount;
        float lastTime;
        float fps;

        private void Start()
        {
            var version = Application.version;
            print(version);
            versionText.text = "AWSIM v " + version;

            frameCount = 0;
            lastTime = 0.0f;
        }

        private void Update()
        {
            frameCount++;
            float time = Time.realtimeSinceStartup - lastTime;

            if (time >= 0.5f)
            {
                fps = frameCount / time;

                frameCount = 0;
                lastTime = Time.realtimeSinceStartup;
            }

            fpsText.text = fps.ToString("F0") + " fps";
        }
    }
}
