using NUnit.Framework;
using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using System.Collections;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

public class VehicleTest
{
    // Scene handling
    AsyncOperation sceneLoader;
    string sceneName = "NPCDriveTest";
    Scene scene;
    AsyncOperation aOp;

    // Shared settings
    float testDuration = 5.0f;

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
        yield return new WaitForSeconds(10);

        LogAssert.Expect(LogType.Log, "Straight");
        
        yield return null;
    }
}
