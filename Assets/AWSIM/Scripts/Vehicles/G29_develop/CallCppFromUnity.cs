using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class CallCppFromUnity : MonoBehaviour
{
    // C++の共有ライブラリにある関数をインポート
    [DllImport("MyCppLibrary")]
    private static extern int Add(int a, int b);

    [DllImport("MyCppLibrary")]
    private static extern float Multiple(float a, float b);

    [DllImport("G29FFB")]
    private static extern bool InitDevice();

    [DllImport("G29FFB")]
    private static extern void UploadEffect(double torque, double attack_length);

    [Range(-0.3f, 0.3f)]
    public float torque = 0;

    [Range(0, 0.5f)]
    public float attackLength = 0;

    void Start()
    {
        // C++の関数を呼び出す
        int result = Add(3, 4);
        Debug.Log("Result from C++ Add: " + result);

        float multipleResult = Multiple(3, 4);
        Debug.Log("Result from C++ Multiple: " + multipleResult);

        Debug.Log(InitDevice());
    }

    void Update()
    {
        UploadEffect((double)torque, (double)attackLength);
    }
}