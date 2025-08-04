using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using System.Diagnostics;
using System.IO;

public class ChangeSrp : EditorWindow
{
    private static readonly string _urpSymbl = "URP";
    private static readonly string _hdrpSymbol = "HDRP";

    private static readonly string _urpAssetPath = "Assets/Awsim/Graphics/URP/New Universal Render Pipeline Asset.asset";
    private static readonly string _hdrpAssetPath = "Assets/Awsim/Graphics/HDRP/HDRenderPipelineAsset.asset";

    [MenuItem("AWSIM/Change Srp")]
    public static void ShowWindow()
    {
        GetWindow<ChangeSrp>("Change Srp");
    }

    void OnGUI()
    {
        GUILayout.Label("Render Pipeline", EditorStyles.boldLabel);

        var currentPipeline = GraphicsSettings.currentRenderPipeline;

        bool isURP = currentPipeline != null && currentPipeline.GetType().Name.Contains("UniversalRenderPipelineAsset");
        bool isHDRP = currentPipeline != null && currentPipeline.GetType().Name.Contains("HDRenderPipelineAsset");

        if (!isURP)
        {
            if (GUILayout.Button("Switch to URP"))
            {
                Switch(_urpAssetPath, _urpSymbl);
            }
        }
        else
        {
            EditorGUILayout.HelpBox("Current pipeline is URP", MessageType.Info);
        }

        if (!isHDRP)
        {
            if (GUILayout.Button("Switch to HDRP"))
            {
                Switch(_hdrpAssetPath, _hdrpSymbol);
            }
        }
        else
        {
            EditorGUILayout.HelpBox("Current pipeline is HDRP", MessageType.Info);
        }
    }

    static void Switch(string pipelineAssetPath, string defineSymbol)
    {
        var asset = AssetDatabase.LoadAssetAtPath<RenderPipelineAsset>(pipelineAssetPath);
        if (asset == null)
        {
            return;
        }

        GraphicsSettings.defaultRenderPipeline = asset;

        string currentDefines = PlayerSettings.GetScriptingDefineSymbolsForGroup(EditorUserBuildSettings.selectedBuildTargetGroup);
        var defineList = new System.Collections.Generic.List<string>(currentDefines.Split(';'));

        defineList.RemoveAll(d => d == "URP" || d == "HDRP");

        if (!defineList.Contains(defineSymbol))
        {
            defineList.Add(defineSymbol);
        }

        string newDefines = string.Join(";", defineList);
        PlayerSettings.SetScriptingDefineSymbolsForGroup(EditorUserBuildSettings.selectedBuildTargetGroup, newDefines);

        bool restart = EditorUtility.DisplayDialog(
            "Restart Unity",
            "Render pipeline has been switched to " + defineSymbol + ".\nWould you like to restart the Unity Editor now?",
            "Yes", "No"
        );

        if (restart)
        {
            EditorApplication.OpenProject(System.IO.Directory.GetCurrentDirectory());
        }
    }
    
}