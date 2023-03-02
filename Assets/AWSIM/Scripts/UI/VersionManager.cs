using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEditor;

public class VersionManager : MonoBehaviour
{

public Text text;

    // Start is called before the first frame update
    void Start()
    {
        var version = Application.version;
        print(version);
        text.text = "AWSIM  v " + version;
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    
}
