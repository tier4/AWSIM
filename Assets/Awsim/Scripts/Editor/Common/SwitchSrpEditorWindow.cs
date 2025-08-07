// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEditor;
using UnityEditor.Build;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Unity Editor window for switching between URP and HDRP render pipelines.
/// This tool updates the RenderPipelineAsset, scripting define symbols,
/// and optionally prompts for restarting the Unity Editor to apply changes.
/// </summary>

public class SwitchSrpEditorWindow : EditorWindow
{
    private static readonly string _urpSymbol = "URP";
    private static readonly string _hdrpSymbol = "HDRP";

    private static readonly string _urpAssetPath = "Assets/Awsim/Graphics/URP/DefaultUrpAsset.asset";
    private static readonly string _hdrpAssetPath = "Assets/Awsim/Graphics/HDRP/DefaultHdrpAsset.asset";

    [MenuItem("AWSIM/Switch SRP")]
    public static void ShowWindow()
    {
        GetWindow<SwitchSrpEditorWindow>("Switch SRP");
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
                Switch(_urpAssetPath, _urpSymbol);
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
            UnityEngine.Debug.LogError($"RenderPipelineAsset not found at: {pipelineAssetPath}");
            return;
        }

        GraphicsSettings.defaultRenderPipeline = asset;

        var target = NamedBuildTarget.FromBuildTargetGroup(EditorUserBuildSettings.selectedBuildTargetGroup);

        PlayerSettings.GetScriptingDefineSymbols(target, out string[] defines);
        var defineList = new System.Collections.Generic.List<string>(defines);

        defineList.RemoveAll(d => d == "URP" || d == "HDRP");

        if (!defineList.Contains(defineSymbol))
        {
            defineList.Add(defineSymbol);
        }

        PlayerSettings.SetScriptingDefineSymbols(target, defineList.ToArray());

        bool restart = EditorUtility.DisplayDialog(
            "Restart Unity",
            $"Render pipeline has been switched to {defineSymbol}.\nWould you like to restart the Unity Editor now?",
            "Yes", "No"
        );

        if (restart)
        {
            EditorApplication.OpenProject(System.IO.Directory.GetCurrentDirectory());
        }
    }

}