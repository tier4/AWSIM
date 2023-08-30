/* 
Copyright 2023 Autonoma, Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:

    http://www.apache.org/licenses/LICENSE-2.0

The software is provided "AS IS", WITHOUT WARRANTY OF ANY KIND, 
express or implied. In no event shall the authors or copyright 
holders be liable for any claim, damages or other liability, 
whether in action of contract, tort or otherwise, arising from, 
out of or in connection with the software or the use of the software.
*/
using System;
using UnityEditor;
using UnityEngine;
using System.IO;

public class CreateAssetBundle
{
    [MenuItem("Assets/Build Asset Bundles")]
    private static void BuildAssetBundles()
    {
        string path = Environment.GetFolderPath(Environment.SpecialFolder.Personal)+"/AWSIM/AssetBundles/";

        if (!Directory.Exists(path+"Windows/")) Directory.CreateDirectory(path+"Windows/");
        if (!Directory.Exists(path+"Linux/")) Directory.CreateDirectory(path+"Linux/");
        if (!Directory.Exists(path+"OSX/")) Directory.CreateDirectory(path+"OSX/");

        try
        {
            BuildPipeline.BuildAssetBundles(path+"Windows/", BuildAssetBundleOptions.ChunkBasedCompression, BuildTarget.StandaloneWindows64);
            BuildPipeline.BuildAssetBundles(path+"Linux/", BuildAssetBundleOptions.ChunkBasedCompression, BuildTarget.StandaloneLinux64);
            BuildPipeline.BuildAssetBundles(path+"OSX/", BuildAssetBundleOptions.ChunkBasedCompression, BuildTarget.StandaloneOSX);
        }
        catch(Exception e)
        {
            Debug.LogWarning(e);
        }
    }
}
