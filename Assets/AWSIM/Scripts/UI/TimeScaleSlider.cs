using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    public class TimeScaleSlider : MonoBehaviour
    {
        [SerializeField] Text timeScaleText;
        [SerializeField] Slider timeScaleSlider;

        private void Start()
        {
            timeScaleText.text = "x " + timeScaleSlider.value.ToString("F2");
        }

        public void SetTimeScale(float timeScale)
        {
            Time.timeScale = timeScale;
            timeScaleText.text = "x " + timeScale.ToString("F2"); 
        }
    }
}