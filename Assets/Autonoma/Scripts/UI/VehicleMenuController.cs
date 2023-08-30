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
using UnityEngine.InputSystem;
using TMPro;
using System.Text.RegularExpressions;
using System;

public class VehicleMenuController : MonoBehaviour
{
    public ScenarioMenuController scenarioMenu;
    public Button mainMenuButton;
    public Button saveVehSetupButton;
    public Button deleteVehSetupButton;
    public Button scenarioSetupButton;
    public Button vehSetupButton;
    public Button sensorSetupButton;
    public Button defaultButton;
    
    public TMP_Dropdown vehSetupDropdown;
    
    public Slider diffInput;
    public Slider idealSteeringInput;
    public TMP_InputField vehSetupNameInput;
    public TMP_InputField delayInput;
    public TMP_InputField bwInput;
    public TMP_InputField angleInput;
    public TMP_InputField rateInput;
    public TMP_InputField frontRollInput;
    public TMP_InputField rearRollInput;
    public TMP_InputField steeringRatioInput;
    public TMP_InputField brakeInput;
    public TMP_InputField tAmbientInput;
    public TMP_InputField tTrackInput;
    public Toggle thermalToggle;
    
    private void Awake() {}

    private void Start() 
    {
        mainMenuButton.onClick.AddListener( GameManager.Instance.UIManager.OnMainMenuPressed );
        saveVehSetupButton.onClick.AddListener( saveVehSetupButtonPressed );
        deleteVehSetupButton.onClick.AddListener ( deleteVehSetupButtonPressed );
        scenarioSetupButton.onClick.AddListener( scenarioSetupButtonPressed );
        vehSetupButton.onClick.AddListener( vehSetupButtonPressed );
        sensorSetupButton.onClick.AddListener( sensorSetupButtonPressed );
        defaultButton.onClick.AddListener ( defaultButtonPressed );
        vehSetupDropdown.onValueChanged.AddListener(delegate { chosenVehSetupChanged(vehSetupDropdown); } );
        idealSteeringInput.onValueChanged.AddListener(delegate { chosenSteeringTypeChanged(idealSteeringInput); } );

        sensorSetupButton.interactable = false;

    }

    private void OnEnable() 
    {
        fillVehSetupDropdown(scenarioMenu.vehSetupDropdown.value);
    
        chosenVehSetupChanged(vehSetupDropdown);
        chosenSteeringTypeChanged(idealSteeringInput);
    }

    private void fillVehSetupDropdown(int idx)
    {   
        vehSetupDropdown.ClearOptions();
        var reversedLoadedVehSetups = scenarioMenu.LoadedVehSetups.ToArray();
        Array.Reverse(reversedLoadedVehSetups);

        foreach(VehSetup vehSetupObj in reversedLoadedVehSetups)
        {
            var op = new TMP_Dropdown.OptionData(vehSetupObj.Name);
            vehSetupDropdown.options.Add(op);
        }
        vehSetupDropdown.value = idx;
        vehSetupDropdown.RefreshShownValue();
    }

    private void chosenVehSetupChanged(TMP_Dropdown dropdown)
    {
        int idx = dropdown.value;
        deleteVehSetupButton.interactable = (idx < 2) ? false : true;
        var reversedLoadedVehSetups = scenarioMenu.LoadedVehSetups.ToArray();
        Array.Reverse(reversedLoadedVehSetups);
        scenarioMenu.tmpVehSetup = reversedLoadedVehSetups[idx];
        
        vehSetupNameInput.text = scenarioMenu.tmpVehSetup.Name;
        diffInput.value = (scenarioMenu.tmpVehSetup.IsLSD) ? 1 : 0;
        idealSteeringInput.value = (scenarioMenu.tmpVehSetup.IsIdealSteering) ? 0 : 1;
        delayInput.text = scenarioMenu.tmpVehSetup.SteeringDelay.ToString();
        bwInput.text = scenarioMenu.tmpVehSetup.SteeringBW.ToString();
        angleInput.text = scenarioMenu.tmpVehSetup.MaxSteeringAngle.ToString();
        rateInput.text = scenarioMenu.tmpVehSetup.MaxSteeringRate.ToString();
        frontRollInput.text = scenarioMenu.tmpVehSetup.FrontRollBarRate.ToString();
        rearRollInput.text = scenarioMenu.tmpVehSetup.RearRollBarRate.ToString();
        steeringRatioInput.text = scenarioMenu.tmpVehSetup.SteeringRatio.ToString();
        brakeInput.text = scenarioMenu.tmpVehSetup.BrakeConstant.ToString();
        thermalToggle.isOn = scenarioMenu.tmpVehSetup.IsThermalTyre;
        tAmbientInput.text = scenarioMenu.tmpVehSetup.AmbientTemp.ToString();
        tTrackInput.text = scenarioMenu.tmpVehSetup.TrackTemp.ToString();
    }

    private void chosenSteeringTypeChanged(Slider slider)
    {
        delayInput.interactable = (slider.value == 0) ? false : true;
        bwInput.interactable = (slider.value == 0) ? false : true;
        angleInput.interactable = (slider.value == 0) ? false : true;
        rateInput.interactable = (slider.value == 0) ? false : true;
    }
    
    private void saveVehSetupButtonPressed()
    {
        int idx = (vehSetupNameInput.text == scenarioMenu.tmpVehSetup.Name) ? vehSetupDropdown.value : scenarioMenu.LoadedVehSetups.Count;

        updateTmpVehSetup();

        saveVehicleSetup(scenarioMenu.tmpVehSetup);

        fillVehSetupDropdown(0);
    }

    private void scenarioSetupButtonPressed()
    {
        updateTmpVehSetup();
        scenarioMenu.fillVehSetupDropdown(vehSetupDropdown.value);
        GameManager.Instance.UIManager.OnScenarioMenuPressed();
    }

    private void vehSetupButtonPressed() {}

    private void sensorSetupButtonPressed()
    {
        updateTmpVehSetup();
        scenarioMenu.fillVehSetupDropdown(vehSetupDropdown.value);
        GameManager.Instance.UIManager.OnSensorSetupMenuPressed();
    }

    private void deleteVehSetupButtonPressed()
    {
        if ( vehSetupDropdown.value > 1 )
        {
            int idx = vehSetupDropdown.value;
            updateTmpVehSetup();
            deleteVehicleSetup(scenarioMenu.tmpVehSetup);
            fillVehSetupDropdown(idx-1);
            chosenVehSetupChanged(vehSetupDropdown);
        }
    }

    private void defaultButtonPressed()
    {
        if ( vehSetupDropdown.value == 0 )
        {
            vehSetupNameInput.text = "Default RC";
            diffInput.value = 1;
            idealSteeringInput.value = 1;
            delayInput.text = "0.01";
            bwInput.text = "5";
            angleInput.text = "240";
            rateInput.text = "500";
            frontRollInput.text = "463593";
            rearRollInput.text = "358225";
            steeringRatioInput.text = "15.0";
            brakeInput.text = "1.0";
            thermalToggle.isOn = true;
            tAmbientInput.text = "20";
            tTrackInput.text = "25";
        }
        else if ( vehSetupDropdown.value == 1 )
        {
            vehSetupNameInput.text = "Default Oval";
            diffInput.value = 0;
            idealSteeringInput.value = 1;
            delayInput.text = "0.01";
            bwInput.text = "5";
            angleInput.text = "200";
            rateInput.text = "360";
            frontRollInput.text = "463593";
            rearRollInput.text = "0";
            steeringRatioInput.text = "19.5";
            brakeInput.text = "1.0";
            thermalToggle.isOn = true;
            tAmbientInput.text = "20";
            tTrackInput.text = "25";
        }
        else
        {
            diffInput.value = 1;
            idealSteeringInput.value = 1;
            delayInput.text = "0.01";
            bwInput.text = "5";
            angleInput.text = "240";
            rateInput.text = "500";
            frontRollInput.text = "463593";
            rearRollInput.text = "358225";
            steeringRatioInput.text = "15.0";
            brakeInput.text = "1.0";
            thermalToggle.isOn = true;
            tAmbientInput.text = "20";
            tTrackInput.text = "25";
        }

        updateTmpVehSetup();
    }

    private void updateTmpVehSetup()
    {   
        scenarioMenu.tmpVehSetup.Name = vehSetupNameInput.text;
        scenarioMenu.tmpVehSetup.IsLSD = (diffInput.value > 0.5) ? true : false;
        scenarioMenu.tmpVehSetup.IsIdealSteering = (idealSteeringInput.value < 0.5) ? true : false;
        scenarioMenu.tmpVehSetup.SteeringDelay = isFFieldValid(delayInput);
        scenarioMenu.tmpVehSetup.SteeringBW = isFFieldValid(bwInput);
        scenarioMenu.tmpVehSetup.MaxSteeringAngle = isFFieldValid(angleInput);
        scenarioMenu.tmpVehSetup.MaxSteeringRate = isFFieldValid(rateInput);
        scenarioMenu.tmpVehSetup.FrontRollBarRate = isFFieldValid(frontRollInput);
        scenarioMenu.tmpVehSetup.RearRollBarRate = isFFieldValid(rearRollInput);
        scenarioMenu.tmpVehSetup.SteeringRatio = isFFieldValid(steeringRatioInput);
        scenarioMenu.tmpVehSetup.BrakeConstant = isFFieldValid(brakeInput);
        scenarioMenu.tmpVehSetup.IsThermalTyre = thermalToggle.isOn;
        scenarioMenu.tmpVehSetup.AmbientTemp = isFFieldValid(tAmbientInput);
        scenarioMenu.tmpVehSetup.TrackTemp = isFFieldValid(tTrackInput);
    }

    private void saveVehicleSetup(VehSetup inputObj)
    {
        SaveDataManager.SaveVehicleSetup(inputObj);
        scenarioMenu.LoadedVehSetups = SaveDataManager.LoadAllVehicleSetups();
    }

    private void deleteVehicleSetup(VehSetup inputObj)
    {
        SaveDataManager.DeleteVehicleSetup(inputObj);
        scenarioMenu.LoadedVehSetups = SaveDataManager.LoadAllVehicleSetups();
    }

    private float isFFieldValid(TMP_InputField input)
    {   float parsedField;
        bool validField = float.TryParse( input.text, out parsedField);
        float outField;
        if (validField)
        {
            outField = parsedField;
        }
        else
        {
            outField = -1;
        }
        return outField;
    }
}