using NUnit.Framework;
using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using System.Collections;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

public class TrafficTest
{
    AsyncOperation sceneLoader;

    [UnitySetUp]
    public IEnumerator Setup()
    {
        yield return EditorSceneManager.LoadSceneAsync("TrafficTest", LoadSceneMode.Additive);
    }

    [UnityTest]
    public IEnumerator Movement()
    {
        Time.timeScale = 2.0f;
        
        yield return new WaitForSeconds(1);
        Debug.Log("Movement");
        
        yield return null;
    }
}
