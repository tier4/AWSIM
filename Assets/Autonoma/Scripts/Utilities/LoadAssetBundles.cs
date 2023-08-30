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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LoadAssetBundles : MonoBehaviour
{
    AssetBundle myLoadedAssetBundle;
    public string path;
    void Start()
    {
        
    }

    // Update is called once per frame
    void LoadAssetBundle(string bundleUrl)
    {
        myLoadedAssetBundle = AssetBundle.LoadFromFile(bundleUrl);

        Debug.Log(myLoadedAssetBundle == null ? " Failed to load AssetBundle" : " AssetBundle successfully loaded");
    }
}
