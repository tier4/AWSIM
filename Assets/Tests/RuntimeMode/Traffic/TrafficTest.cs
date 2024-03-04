using NUnit.Framework;
using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using System.Collections;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

public class TrafficTest
{
    // Scene handling
    AsyncOperation sceneLoader;
    string sceneName = "TrafficTest";
    Scene scene;
    AsyncOperation aOp;

    [OneTimeSetUp]
    public void OneTimeSetUp()
    {
        // Scene async operation
        aOp = EditorSceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);
    }

    [UnitySetUp]
    public IEnumerator Setup()
    {
        yield return new WaitUntil(() => aOp.isDone);
        scene = EditorSceneManager.GetSceneByName(sceneName);
        EditorSceneManager.SetActiveScene(scene);

        Assert.NotNull(scene);
        yield return null;
    }


    [UnityTest]
    public IEnumerator Movement()
    {
        Time.timeScale = 2.0f;
        
        yield return new WaitForSeconds(1);
        Debug.Log("Movement");
        
        yield return null;
    }

    [OneTimeTearDown]
    public void OneTimeTearDown()
    {
        EditorSceneManager.UnloadScene(scene);
    }
}
