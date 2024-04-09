using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    public class DemoUI : MonoBehaviour
    {
        [SerializeField] Text versionText;

        private void Start()
        {
            var version = Application.version;
            print(version);
            versionText.text = "AWSIM v " + version;
        }
    }
}
