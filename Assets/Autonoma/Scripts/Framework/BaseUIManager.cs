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
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public abstract class BaseUIManager : MonoBehaviour
{
    protected abstract void CustomUpdate();
    public abstract void OnMainMenuPressed();
    public abstract void OnScenarioMenuPressed();
    public abstract void OnVehicleSetupMenuPressed();

    public abstract void OnSensorSetupMenuPressed();
    public abstract void OnLicenseMenuPressed();
    public abstract void OnPauseMenuPressed();
    public abstract void OnResumePressed();
    public abstract void RestartScenarioPressed();

    private void Update()
    {
        CustomUpdate();
    }

    public void OnQuitPressed()
    {
        Debug.Log("Shutting down application!");
        if (Application.isEditor)
        {
            #if UNITY_EDITOR
            EditorApplication.ExitPlaymode();
            #endif
        }
        else
        {
            Application.Quit();
        }
    }
}