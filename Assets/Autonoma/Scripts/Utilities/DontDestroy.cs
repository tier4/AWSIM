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

public class DontDestroy : MonoBehaviour
{
    private void Awake()
    {
        // Check if there's another instance of this script in the scene.
        DontDestroy[] instances = FindObjectsOfType<DontDestroy>();

        // If there's more than one instance, destroy this one.
        if (instances.Length > 1)
        {
            Destroy(gameObject);
            return;
        }

        // Mark this object as persistent between scenes.
        DontDestroyOnLoad(gameObject);
    }
}
