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
using UnityEngine.SceneManagement;
using UnityEngine;

public class SimUIManager : BaseUIManager
{
    public Canvas DriveMenu;
    public Canvas PauseMenu;
    // public List<GameObject> rosCars;

    protected override void CustomUpdate()
    {
        switch (GameManager.Instance.Status)
        {
            case GameManager.SimulationState.DRIVE:
            {
                DriveMenu.gameObject.SetActive(true);
                PauseMenu.gameObject.SetActive(false);
                break;
            }
                case GameManager.SimulationState.PAUSE:
            {
                DriveMenu.gameObject.SetActive(false);
                PauseMenu.gameObject.SetActive(true);
                break;
            }
        }
    }

    public override void OnPauseMenuPressed()
    {
        GameManager.Instance.ChangeStateTo(GameManager.SimulationState.PAUSE);    
    }

    public override void OnResumePressed()
    {
        GameManager.Instance.ChangeStateTo(GameManager.SimulationState.DRIVE);    
    }
    
    public override void RestartScenarioPressed()
    {
        GameManager.Instance.ChangeStateTo(GameManager.SimulationState.RESTART);
    }

    public override void OnMainMenuPressed()
    {
        GameManager.Instance.StartCoroutine(GameManager.Instance.ChangeScene("MenuScene"));
        GameManager.Instance.ChangeStateTo(GameManager.SimulationState.MENU);
    }

    public override void OnLicenseMenuPressed() {}

    public override void OnScenarioMenuPressed() {}

    public override void OnVehicleSetupMenuPressed() {}

    public override void OnSensorSetupMenuPressed() {}
}