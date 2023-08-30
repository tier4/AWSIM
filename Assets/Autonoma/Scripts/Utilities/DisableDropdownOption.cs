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
using UnityEngine.UI;

public class DisableDropdownOption : MonoBehaviour
{
    public List<string> optionsToDisable = new List<string>();
    void Start () 
    {
        //This script should be attached to Item
        Toggle toggle = gameObject.GetComponent<Toggle>();
        // Uncomment this to see the item options (ex: Item 1: ROS)
        // Debug.Log(toggle);
        if (toggle != null && optionsToDisable.Contains(toggle.name))
        {
            toggle.interactable = false;
        }
        else
        {
            toggle.interactable = true;
        }
    }
}
