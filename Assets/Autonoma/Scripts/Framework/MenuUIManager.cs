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

public class MenuUIManager : BaseUIManager
{
    public Canvas MainMenu;
    public Canvas ScenarioMenu;
    public Canvas VehicleSetupMenu;
    public Canvas SensorSetupMenu;
    public Canvas LicenseMenu;
    private enum MenuCanvas
    {
        MAINMENU,
        SCENARIO,
        VEHICLESETUP,
        SENSORSETUP,
        LICENSE,
    }
    private MenuCanvas currentCanvas;

    private void Start() 
    {
        currentCanvas = MenuCanvas.MAINMENU;
    }
    protected override void CustomUpdate()
    {
        switch(currentCanvas)
        {
            case MenuCanvas.MAINMENU:
            MainMenu.gameObject.SetActive(true);
            ScenarioMenu.gameObject.SetActive(false);
            VehicleSetupMenu.gameObject.SetActive(false);
            SensorSetupMenu.gameObject.SetActive(false);
            LicenseMenu.gameObject.SetActive(false);
            break;

            case MenuCanvas.SCENARIO:
            MainMenu.gameObject.SetActive(false);
            ScenarioMenu.gameObject.SetActive(true);
            VehicleSetupMenu.gameObject.SetActive(false);
            SensorSetupMenu.gameObject.SetActive(false);
            LicenseMenu.gameObject.SetActive(false);
            break;

            case MenuCanvas.VEHICLESETUP:
            MainMenu.gameObject.SetActive(false);
            ScenarioMenu.gameObject.SetActive(true);
            VehicleSetupMenu.gameObject.SetActive(true);
            SensorSetupMenu.gameObject.SetActive(false);
            LicenseMenu.gameObject.SetActive(false);
            break;

            case MenuCanvas.SENSORSETUP:
            MainMenu.gameObject.SetActive(false);
            ScenarioMenu.gameObject.SetActive(true);
            VehicleSetupMenu.gameObject.SetActive(false);
            SensorSetupMenu.gameObject.SetActive(true);
            LicenseMenu.gameObject.SetActive(false);
            break;

            case MenuCanvas.LICENSE:
            MainMenu.gameObject.SetActive(false);
            ScenarioMenu.gameObject.SetActive(false);
            VehicleSetupMenu.gameObject.SetActive(false);
            SensorSetupMenu.gameObject.SetActive(false);
            LicenseMenu.gameObject.SetActive(true);
            break;
        }
    }

    public override void OnMainMenuPressed()
    {
        currentCanvas = MenuCanvas.MAINMENU;
    }

    public override void OnLicenseMenuPressed()
    {
        currentCanvas = MenuCanvas.LICENSE;
    }

    public override void OnScenarioMenuPressed()
    {
        currentCanvas = MenuCanvas.SCENARIO;
    }

    public override void OnVehicleSetupMenuPressed()
    {
        currentCanvas = MenuCanvas.VEHICLESETUP;
    }

    public override void OnSensorSetupMenuPressed()
    {
        currentCanvas = MenuCanvas.SENSORSETUP;
    }

    public override void OnPauseMenuPressed() {}

    public override void OnResumePressed() {}
    
    public override void RestartScenarioPressed() {}
}