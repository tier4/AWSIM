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

public class SensorMenuController : MonoBehaviour
{
    public ScenarioMenuController scenarioMenu;
    internal List<SensorPrefabController> prefabList = new List<SensorPrefabController>();
    public Button mainMenuButton;
    public Button saveSensorSetButton;
    public Button deleteSensorSetButton;
    public Button scenarioSetupButton;
    public Button sensorSetupButton;
    public Button vehSetupButton;
    public Button addSensorButton;
    
    public TMP_Dropdown sensorSetDropdown;
    
    public TMP_InputField sensorSetNameInput;
    
    private void Awake() {}

    private void Start() 
    {
        mainMenuButton.onClick.AddListener( GameManager.Instance.UIManager.OnMainMenuPressed );
        saveSensorSetButton.onClick.AddListener( saveSensorSetButtonPressed );
        deleteSensorSetButton.onClick.AddListener ( deleteSensorSetButtonPressed );
        scenarioSetupButton.onClick.AddListener( scenarioSetupButtonPressed );
        vehSetupButton.onClick.AddListener( vehSetupButtonPressed );
        sensorSetupButton.onClick.AddListener( sensorSetupButtonPressed );
        addSensorButton.onClick.AddListener( addSensorButtonPressed  );
        sensorSetDropdown.onValueChanged.AddListener(delegate { chosenSensorSetChanged(sensorSetDropdown); } );
    }

    private void OnEnable() 
    {
        SensorPrefabController.OnSensorDeleted += handleSensorDeleted;
        
        if (scenarioMenu == null)
        {
            Debug.LogWarning("ScenarioMenuController instance is null");
        }
        else
        {
            fillSensorSetDropdown(scenarioMenu.sensorSetDropdown.value);
            chosenSensorSetChanged(sensorSetDropdown);
            saveSensorSetButtonPressed();
        }
    }

    private void OnDisable()
    {
        SensorPrefabController.OnSensorDeleted -= handleSensorDeleted;
    }

    private void fillSensorSetDropdown(int idx)
    {   
        sensorSetDropdown.ClearOptions(); 
        var reversedLoadedSensorSets = scenarioMenu.LoadedSensorSets.ToArray();
        Array.Reverse(reversedLoadedSensorSets);

        foreach(SensorSet sensorSetObj in reversedLoadedSensorSets)
        {
            var op = new TMP_Dropdown.OptionData(sensorSetObj.Name);
            sensorSetDropdown.options.Add(op);
        }
        sensorSetDropdown.value = idx;
        sensorSetDropdown.RefreshShownValue();
    }

    private void chosenSensorSetChanged(TMP_Dropdown dropdown)
    {
        int idx = dropdown.value;
        var reversedLoadedSensorSets = scenarioMenu.LoadedSensorSets.ToArray();
        Array.Reverse(reversedLoadedSensorSets);
        deleteSensorSetButton.interactable = (scenarioMenu.LoadedSensorSets.Count <= 1) ? false : true;
        scenarioMenu.tmpSensorSet = reversedLoadedSensorSets[idx];
        
        sensorSetNameInput.text = scenarioMenu.tmpSensorSet.Name;
    }

    private void saveSensorSetButtonPressed()
    {
        int idx = (sensorSetNameInput.text == scenarioMenu.tmpSensorSet.Name) ? sensorSetDropdown.value : scenarioMenu.LoadedSensorSets.Count;

        updateTmpSensorSet();

        saveSensorSet(scenarioMenu.tmpSensorSet);

        fillSensorSetDropdown(0);
    }

    private void scenarioSetupButtonPressed()
    {
        updateTmpSensorSet();
        scenarioMenu.fillSensorSetDropdown(sensorSetDropdown.value);
        GameManager.Instance.UIManager.OnScenarioMenuPressed();
    }

    private void vehSetupButtonPressed()
    {
        updateTmpSensorSet();
        scenarioMenu.fillSensorSetDropdown(sensorSetDropdown.value);
        GameManager.Instance.UIManager.OnVehicleSetupMenuPressed();
    }

    private void sensorSetupButtonPressed() {}

    private void deleteSensorSetButtonPressed()
    {
        if ( sensorSetDropdown.value > 0 )
        {
            int idx = sensorSetDropdown.value;
            updateTmpSensorSet();
            deleteSensorSet(scenarioMenu.tmpSensorSet);
            fillSensorSetDropdown(idx-1);
            chosenSensorSetChanged(sensorSetDropdown);
        }
    }

    private void addSensorButtonPressed()
    {
        // Add sensor logic
    }

    private void updateTmpSensorSet()
    {   
        // Put all input data to tmpSensorSet object
        scenarioMenu.tmpSensorSet.Name = sensorSetNameInput.text;
    }

    private void saveSensorSet(SensorSet inputObj)
    {
        SaveDataManager.SaveSensorSet(inputObj);
        scenarioMenu.LoadedSensorSets = SaveDataManager.LoadAllSensorSets();
    }

    private void deleteSensorSet(SensorSet inputObj)
    {
        SaveDataManager.DeleteSensorSet(inputObj);
        scenarioMenu.LoadedSensorSets = SaveDataManager.LoadAllSensorSets();
    }

    private void handleSensorDeleted(SensorPrefabController prefab)
    {
        prefabList.Remove(prefab);
    }
}