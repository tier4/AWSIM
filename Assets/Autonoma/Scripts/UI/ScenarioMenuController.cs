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

public class ScenarioMenuController : MonoBehaviour
{
    public List<ScenarioObj> LoadedScenarioObjs;
    public List<VehSetup> LoadedVehSetups;
    public List<SensorSet> LoadedSensorSets;
    public TrackList LoadedTrackList;
    public ScenarioObj tmpScenarioObj = new ScenarioObj();
    public VehSetup tmpVehSetup = new VehSetup();
    public SensorSet tmpSensorSet = new SensorSet();

    public Button mainMenuButton;
    public Button saveScenarioButton;
    public Button deleteScenarioButton;
    public Button scenarioSetupButton;
    public Button vehSetupButton;
    public Button sensorSetupButton;
    public Button driveButton;

    public TMP_Dropdown scenarioDropdown;
    public TMP_Dropdown vehSetupDropdown;
    public TMP_Dropdown sensorSetDropdown;
    public TMP_Dropdown trackDropdown; 
    
    public Toggle hotStartToggle;
    public TMP_InputField scenarioNameInput;
    public Slider numCarsInput;
    public TMP_Dropdown[] controlTypeDropdowns = new TMP_Dropdown[3]; 
    public TMP_Dropdown[] colorDropdowns = new TMP_Dropdown[3];
    public TMP_InputField[] carSpawnPosInput = new TMP_InputField[3];
    public TMP_InputField[] carNumInput = new TMP_InputField[3];
    public TMP_InputField[] rosDomainInput = new TMP_InputField[3];
    
    private void Awake()
    {  
        mainMenuButton.onClick.AddListener( GameManager.Instance.UIManager.OnMainMenuPressed );
        saveScenarioButton.onClick.AddListener( saveScenarioButtonPressed );
        scenarioSetupButton.onClick.AddListener( scenarioSetupButtonPressed );
        vehSetupButton.onClick.AddListener( vehSetupButtonPressed );
        sensorSetupButton.onClick.AddListener( sensorSetupButtonPressed );
        deleteScenarioButton.onClick.AddListener ( deleteScenarioButtonPressed );
        driveButton.onClick.AddListener( driveButtonPressed  );
    }

    private void Start() 
    {
        LoadedScenarioObjs = SaveDataManager.LoadAllScenarios();
        LoadedVehSetups = SaveDataManager.LoadAllVehicleSetups();
        LoadedSensorSets = SaveDataManager.LoadAllSensorSets();

        fillTrackDropdown(0);
        checkForDefaultScenario();
        checkForDefaultVehSetups();
        checkForDefaultSensorSets();

        fillControlTypeDropdown(0,0);
        fillControlTypeDropdown(0,1);
        fillControlTypeDropdown(0,2);
        fillColorDropdown(0,0);
        fillColorDropdown(0,1);
        fillColorDropdown(0,2);

        scenarioDropdown.onValueChanged.AddListener(delegate { chosenScenarioChanged(scenarioDropdown); } );
        vehSetupDropdown.onValueChanged.AddListener(delegate { chosenVehSetupChanged(vehSetupDropdown); } );
        sensorSetDropdown.onValueChanged.AddListener(delegate { chosenSensorSetChanged(sensorSetDropdown); } );
        trackDropdown.onValueChanged.AddListener(delegate { chosenTrackChanged(trackDropdown); } );
        numCarsInput.onValueChanged.AddListener(delegate { chosenNumCarsChanged(numCarsInput); } );
        
        chosenScenarioChanged(scenarioDropdown);
        chosenVehSetupChanged(vehSetupDropdown);
        chosenSensorSetChanged(sensorSetDropdown);
        chosenNumCarsChanged(numCarsInput);

        trackDropdown.value = tmpScenarioObj.SelectedTrack;
        trackDropdown.RefreshShownValue();
        saveScenarioButtonPressed();

        sensorSetupButton.interactable = false;
    }

    private void OnEnable() {}

    private void fillScenarioDropdown(int idx)
    {   
        scenarioDropdown.ClearOptions(); 
        var reversedLoadedScenarioObjs = LoadedScenarioObjs.ToArray();
        Array.Reverse(reversedLoadedScenarioObjs);

        foreach(ScenarioObj scenarioObj in reversedLoadedScenarioObjs)
        {
            var op = new TMP_Dropdown.OptionData(scenarioObj.Name);
            scenarioDropdown.options.Add(op);
        }
        scenarioDropdown.value = idx;
        scenarioDropdown.RefreshShownValue();
    }

    public void fillVehSetupDropdown(int idx)
    {   
        vehSetupDropdown.ClearOptions();
        var reversedLoadedVehSetups = LoadedVehSetups.ToArray();
        Array.Reverse(reversedLoadedVehSetups);

        foreach(VehSetup vehSetupObj in reversedLoadedVehSetups)
        {
            var op = new TMP_Dropdown.OptionData(vehSetupObj.Name);
            vehSetupDropdown.options.Add(op);
        }
        vehSetupDropdown.value = idx;
        vehSetupDropdown.RefreshShownValue();
    }

    public void fillSensorSetDropdown(int idx)
    {   
        sensorSetDropdown.ClearOptions();
        var reversedLoadedSensorSets = LoadedSensorSets.ToArray();
        Array.Reverse(reversedLoadedSensorSets);

        foreach(SensorSet sensorSetObj in reversedLoadedSensorSets)
        {
            var op = new TMP_Dropdown.OptionData(sensorSetObj.Name);
            sensorSetDropdown.options.Add(op);
        }
        sensorSetDropdown.value = idx;
        sensorSetDropdown.RefreshShownValue();
    }

    void fillTrackDropdown(int idx)
    {   
        trackDropdown.ClearOptions(); 
        foreach(TrackParams track in LoadedTrackList.MyTrackList)
        {
            var op = new TMP_Dropdown.OptionData(track.TrackName);
            trackDropdown.options.Add(op);
        }
        trackDropdown.value = idx;
        trackDropdown.RefreshShownValue();
    }

    private void fillControlTypeDropdown(int controlIdx,int carIdx)
    {   
        controlTypeDropdowns[carIdx].ClearOptions(); 
        foreach(string controlTypeName in System.Enum.GetNames(typeof(ControlType)))
        {
            var op = new TMP_Dropdown.OptionData(controlTypeName);
            controlTypeDropdowns[carIdx].options.Add(op);
        }
        controlTypeDropdowns[carIdx].value = controlIdx;
        controlTypeDropdowns[carIdx].RefreshShownValue();
    }

    private void fillColorDropdown(int controlIdx,int carIdx)
    {   
        colorDropdowns[carIdx].ClearOptions(); 
        foreach(string colorName in System.Enum.GetNames(typeof(BodyColor)))
        {
            var op = new TMP_Dropdown.OptionData(colorName);
            colorDropdowns[carIdx].options.Add(op);
        }
        colorDropdowns[carIdx].value = controlIdx;
        colorDropdowns[carIdx].RefreshShownValue();
    }
 
    private void chosenTrackChanged(TMP_Dropdown dropdown)
    {
        int idx = dropdown.value;
        tmpScenarioObj.SelectedTrack = idx;
    }

    private void chosenScenarioChanged(TMP_Dropdown dropdown)
    {
        int idx = dropdown.value;
        deleteScenarioButton.interactable = (LoadedScenarioObjs.Count <= 1) ? false : true;
        var reversedLoadedScenarioObjs = LoadedScenarioObjs.ToArray();
        Array.Reverse(reversedLoadedScenarioObjs);
        tmpScenarioObj = reversedLoadedScenarioObjs[idx];
        
        scenarioNameInput.text = tmpScenarioObj.Name;
        numCarsInput.value =  tmpScenarioObj.NumCars;
        trackDropdown.value = tmpScenarioObj.SelectedTrack;


        for (int i = 0; i <tmpScenarioObj.Cars.Count; i++)
        {
            carSpawnPosInput[i].text = tmpScenarioObj.Cars[i].SpawnPositionIdx.ToString();
            carNumInput[i].text = tmpScenarioObj.Cars[i].CarNum.ToString();
            controlTypeDropdowns[i].value = (int)tmpScenarioObj.Cars[i].ControlType;
            colorDropdowns[i].value = (int)tmpScenarioObj.Cars[i].Color;
            rosDomainInput[i].text = tmpScenarioObj.Cars[i].RosDomain.ToString();
        }
    }

    private void chosenVehSetupChanged(TMP_Dropdown dropdown)
    {
        int idx = dropdown.value;
        var reversedLoadedVehSetups = LoadedVehSetups.ToArray();
        Array.Reverse(reversedLoadedVehSetups);
        tmpVehSetup = reversedLoadedVehSetups[idx];
    }

    private void chosenSensorSetChanged(TMP_Dropdown dropdown)
    {
        int idx = dropdown.value;
        var reversedLoadedSensorSets = LoadedSensorSets.ToArray();
        Array.Reverse(reversedLoadedSensorSets);
        tmpSensorSet = reversedLoadedSensorSets[idx];
    }

    private void chosenNumCarsChanged(Slider slider)
    {
        for (int i = 0; i < tmpScenarioObj.Cars.Count; i++)
        {
            carSpawnPosInput[i].interactable = (i >= slider.value) ? false : true;
            carNumInput[i].interactable = (i >= slider.value) ? false : true;
            controlTypeDropdowns[i].interactable = (i >= slider.value) ? false : true;
            colorDropdowns[i].interactable = (i >= slider.value) ? false : true;
            rosDomainInput[i].interactable = (i >= slider.value) ? false : true;
        }    
    }

    private void chosenControlTypeChanged(int carIdx) {}

    private void chosenColorChanged(int carIdx) {}
    
    private void saveScenarioButtonPressed()
    {   
        int idx = (scenarioNameInput.text == tmpScenarioObj.Name) ? scenarioDropdown.value : LoadedScenarioObjs.Count;
        
        updateTmpScenario();

        saveScenario(tmpScenarioObj);
        
        fillScenarioDropdown(0);
    }

    private void scenarioSetupButtonPressed() {}

    private void vehSetupButtonPressed()
    {
        updateTmpScenario();
        GameManager.Instance.UIManager.OnVehicleSetupMenuPressed();
    }

    private void sensorSetupButtonPressed()
    {
        updateTmpScenario();
        GameManager.Instance.UIManager.OnSensorSetupMenuPressed();
    }

    private void deleteScenarioButtonPressed()
    {   
        if ( scenarioDropdown.value > 0 )
        {
            int idx = scenarioDropdown.value;
            updateTmpScenario();
            deleteScenario(tmpScenarioObj);
            fillScenarioDropdown(idx-1);
            chosenScenarioChanged(scenarioDropdown);
        }
    }

    private void checkForDefaultScenario()
    {
        if (LoadedScenarioObjs.Count <= 1 && scenarioNameInput.text == "")
        {
            scenarioNameInput.text = "Default";
            numCarsInput.value =  1;
            trackDropdown.value = 0;
            hotStartToggle.isOn = true;
            carSpawnPosInput[0].text = "0";
            carNumInput[0].text = "1";
            rosDomainInput[0].text = "0";
            carSpawnPosInput[1].text = "1";
            carNumInput[1].text = "0";
            rosDomainInput[1].text = "1";
            carSpawnPosInput[2].text = "2";
            carNumInput[2].text = "0";
            rosDomainInput[2].text = "2";
            fillControlTypeDropdown(0,0);
            fillControlTypeDropdown(0,1);
            fillControlTypeDropdown(0,2); 
            fillColorDropdown(0,0);
            fillColorDropdown(0,1);
            fillColorDropdown(0,2); 

            updateTmpScenario();

            saveScenario(tmpScenarioObj);
        }

        fillScenarioDropdown(0);
    }

    private void checkForDefaultVehSetups()
    {
        if (LoadedVehSetups.Count < 2)
        {
            tmpVehSetup.Name = "Default RC";
            tmpVehSetup.IsLSD = true;
            tmpVehSetup.IsIdealSteering = false;
            tmpVehSetup.SteeringDelay = 0.01f;
            tmpVehSetup.SteeringBW = 5f;
            tmpVehSetup.MaxSteeringAngle = 240f;
            tmpVehSetup.MaxSteeringRate = 500f;
            tmpVehSetup.FrontRollBarRate = 463593f;
            tmpVehSetup.RearRollBarRate = 358225f;
            tmpVehSetup.SteeringRatio = 15f;
            tmpVehSetup.BrakeConstant = 1f;
            tmpVehSetup.IsThermalTyre = true;
            tmpVehSetup.AmbientTemp = 20f;
            tmpVehSetup.TrackTemp = 25f;

            saveVehicleSetup(tmpVehSetup);

            tmpVehSetup.Name = "Default Oval";
            tmpVehSetup.IsLSD = false;
            tmpVehSetup.IsIdealSteering = false;
            tmpVehSetup.SteeringDelay = 0.01f;
            tmpVehSetup.SteeringBW = 5f;
            tmpVehSetup.MaxSteeringAngle = 200f;
            tmpVehSetup.MaxSteeringRate = 360f;
            tmpVehSetup.FrontRollBarRate = 463593f;
            tmpVehSetup.RearRollBarRate = 0f;
            tmpVehSetup.SteeringRatio = 19.5f;
            tmpVehSetup.BrakeConstant = 1f;
            tmpVehSetup.IsThermalTyre = true;
            tmpVehSetup.AmbientTemp = 20f;
            tmpVehSetup.TrackTemp = 25f;

            saveVehicleSetup(tmpVehSetup);
        }

        fillVehSetupDropdown(0);
    }

    private void checkForDefaultSensorSets()
    {
        if (LoadedSensorSets.Count < 1)
        {
            tmpSensorSet.Name = "Default IAC";

            // Novatel
            ISensor novatel = SensorFactory.CreateGnssInsSensor();
            novatel.IsActive = true;
            novatel.TopicNamespace = "novatel_bottom";
            novatel.Translation = new Vector3(0f, 0f, 0f);
            novatel.Rotation = new Vector3(0f, 0f, 0f);
            (novatel.Options as GnssInsSensorOptions).Model = SensorModel.GnssIns.PWRPAK7;
            tmpSensorSet.SensorList.Add(novatel);

            // Luminar
            ISensor lidarSensor = SensorFactory.CreateLidarSensor();
            lidarSensor.IsActive = false;
            lidarSensor.TopicNamespace = "luminar";
            lidarSensor.Translation = new Vector3(0f, 0f, 0f);
            lidarSensor.Rotation = new Vector3(0f, 0f, 0f);
            (lidarSensor.Options as LidarSensorOptions).Model = SensorModel.Lidar.LUMINAR;
            tmpSensorSet.SensorList.Add(lidarSensor);

            // Camera
            ISensor cameraSensor = SensorFactory.CreateCameraSensor();
            cameraSensor.IsActive = false;
            cameraSensor.TopicNamespace = "camera";
            cameraSensor.Translation = new Vector3(0f, 0f, 0f);
            cameraSensor.Rotation = new Vector3(0f, 0f, 0f);
            tmpSensorSet.SensorList.Add(cameraSensor);

            // Raptor sensor
            ISensor raptorSensor = SensorFactory.CreateRaptorSensor();
            raptorSensor.IsActive = true;
            tmpSensorSet.SensorList.Add(raptorSensor);

            // Ground truth sensor
            ISensor gtSensor = SensorFactory.CreateGroundTruthSensor();
            gtSensor.IsActive = true;
            tmpSensorSet.SensorList.Add(gtSensor);

            saveSensorSet(tmpSensorSet);
        }

        fillSensorSetDropdown(0);
    }

    private void driveButtonPressed()
    {
        updateTmpScenario();
        GameManager.Instance.Settings.myScenarioObj = tmpScenarioObj;
        GameManager.Instance.Settings.myVehSetup = tmpVehSetup;
        GameManager.Instance.Settings.mySensorSet = tmpSensorSet;
        GameManager.Instance.Settings.myTrackParams = LoadedTrackList.MyTrackList[tmpScenarioObj.SelectedTrack];
        GameManager.Instance.ChangeStateTo(GameManager.SimulationState.DRIVE);
        GameManager.Instance.StartCoroutine(GameManager.Instance.ChangeScene("DrivingScene"));
    }

    private void updateTmpScenario()
    {   
        int[] spawnPosParsed = new int[3];
        int[] carNumParsed = new int[3];
        spawnPosParsed[0] = isIntFieldValid(carSpawnPosInput[0]);
        spawnPosParsed[1] = isIntFieldValid(carSpawnPosInput[1]);
        spawnPosParsed[2] = isIntFieldValid(carSpawnPosInput[2]);
        carNumParsed[0] = isIntFieldValid(carNumInput[0]);
        carNumParsed[1] = isIntFieldValid(carNumInput[1]);
        carNumParsed[2] = isIntFieldValid(carNumInput[2]);

        int idx = 0;
        for (int i  =0; i< 3; i++)
        {
            if (spawnPosParsed[i] == -1)
            {
                spawnPosParsed[i] = idx;
                idx++;
            }
        }
        
        tmpScenarioObj.Name = scenarioNameInput.text;
        tmpScenarioObj.NumCars = (int)numCarsInput.value;
        tmpScenarioObj.SelectedTrack = trackDropdown.value;
        tmpScenarioObj.HotStart = hotStartToggle.isOn;
        tmpScenarioObj.Cars = new List<Car>()
        {
            new Car()
            {
            ControlType =  (ControlType)controlTypeDropdowns[0].value,
            Color =  (BodyColor)colorDropdowns[0].value,
            CarNum = carNumParsed[0],
            SpawnPositionIdx = spawnPosParsed[0],
            CameraPriority = 10,  
            RosDomain = isIntFieldValid(rosDomainInput[0]),
            },

            new Car()
            {
            ControlType =  (ControlType)controlTypeDropdowns[1].value,
            Color =  (BodyColor)colorDropdowns[1].value,
            CarNum = carNumParsed[1],
            SpawnPositionIdx = spawnPosParsed[1],
            CameraPriority = 10,
            RosDomain = isIntFieldValid(rosDomainInput[1]),
            },
            
            new Car()
            {
            ControlType =  (ControlType)controlTypeDropdowns[2].value,
            Color =  (BodyColor)colorDropdowns[2].value,
            CarNum = carNumParsed[2],
            SpawnPositionIdx = spawnPosParsed[2],
            CameraPriority = 10,
            RosDomain = isIntFieldValid(rosDomainInput[2]),
            },
        };
    }   

    private void saveScenario(ScenarioObj inputObj)
    {
        SaveDataManager.SaveScenario(inputObj);
        LoadedScenarioObjs = SaveDataManager.LoadAllScenarios();
    }

    private void deleteScenario(ScenarioObj inputObj)
    {
        SaveDataManager.DeleteScenario(inputObj);
        LoadedScenarioObjs = SaveDataManager.LoadAllScenarios();
    }

    private void saveVehicleSetup(VehSetup inputObj)
    {
        SaveDataManager.SaveVehicleSetup(inputObj);
        LoadedVehSetups = SaveDataManager.LoadAllVehicleSetups();
    }

    private void saveSensorSet(SensorSet inputObj)
    {
        SaveDataManager.SaveSensorSet(inputObj);
        LoadedSensorSets = SaveDataManager.LoadAllSensorSets();
    }

    private int isIntFieldValid(TMP_InputField input)
    {   int parsedField;
        bool validField = int.TryParse( input.text, out parsedField);
        int outField;
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