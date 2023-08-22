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

public class SensorPrefabController : MonoBehaviour
{
    private SensorMenuController sensorMenu;
    public SensorPrefabController(SensorMenuController menuObj)
    {
        sensorMenu = menuObj;
    }

    public TMP_Text Number;
    public Toggle enabledToggle;
    public TMP_Dropdown sensorTypeDropdown;
    public TMP_Dropdown linkNameDropdown;
    public TMP_Dropdown sensorModelDropdown;
    public TMP_InputField namespaceInputField;
    public Button optionsButton;
    public Button deleteButton;
    
    public GameObject optionsPanel;
    
    public ISensor sensor = SensorFactory.CreateGroundTruthSensor();
    public delegate void SensorDestroyedAction(SensorPrefabController prefab);
    public static event SensorDestroyedAction OnSensorDeleted;
    
    private void Awake() {}

    private void Start() 
    {
        sensor.IsActive = true;
        sensor.SensorLink = "base_link";
        sensor.TopicNamespace = "ground_truth";
        sensor.Translation = new Vector3(0f, 0f, 0f);
        sensor.Rotation = new Vector3(0f, 0f, 0f);

        sensorTypeDropdown.onValueChanged.AddListener(delegate { chosenSensorTypeChanged(sensorTypeDropdown); } );
        linkNameDropdown.onValueChanged.AddListener(delegate { chosenLinkNameChanged(linkNameDropdown); } );
        sensorModelDropdown.onValueChanged.AddListener(delegate { chosenSensorModelChanged(sensorModelDropdown); } );
        namespaceInputField.onEndEdit.AddListener(delegate { namespaceChanged(namespaceInputField); } );
        optionsButton.onClick.AddListener( optionsButtonPressed );
        deleteButton.onClick.AddListener( deleteButtonPressed  );
    }

    private void OnEnable() {}

    private void chosenSensorTypeChanged(TMP_Dropdown dropdown)
    {
        GameObject genericOptionsGroup = optionsPanel.transform.Find("GenericOptionsGroup").GetComponent<GameObject>();
        GameObject gnssInsOptionsGroup = optionsPanel.transform.Find("GnssInsOptionsGroup").GetComponent<GameObject>();
        GameObject raptorOptionsGroup = optionsPanel.transform.Find("RaptorOptionsGroup").GetComponent<GameObject>();

        int idx = dropdown.value;

        switch ((SensorType)idx)
        {
            case SensorType.GNSS_INS:
            sensor = SensorFactory.CreateGnssInsSensor();
            genericOptionsGroup.gameObject.SetActive(false);
            gnssInsOptionsGroup.gameObject.SetActive(true);
            raptorOptionsGroup.gameObject.SetActive(false);
            // fill sensor model dropdown based on this
            break;

            case SensorType.LIDAR:
            sensor = SensorFactory.CreateLidarSensor();
            genericOptionsGroup.gameObject.SetActive(true);
            gnssInsOptionsGroup.gameObject.SetActive(false);
            raptorOptionsGroup.gameObject.SetActive(false);
            break;

            case SensorType.CAMERA:
            sensor = SensorFactory.CreateCameraSensor();
            genericOptionsGroup.gameObject.SetActive(true);
            gnssInsOptionsGroup.gameObject.SetActive(false);
            raptorOptionsGroup.gameObject.SetActive(false);
            break;

            case SensorType.RADAR:
            sensor = SensorFactory.CreateRadarSensor();
            genericOptionsGroup.gameObject.SetActive(true);
            gnssInsOptionsGroup.gameObject.SetActive(false);
            raptorOptionsGroup.gameObject.SetActive(false);
            break;

            case SensorType.RAPTOR:
            sensor = SensorFactory.CreateRaptorSensor();
            genericOptionsGroup.gameObject.SetActive(false);
            gnssInsOptionsGroup.gameObject.SetActive(false);
            raptorOptionsGroup.gameObject.SetActive(true);
            break;

            case SensorType.GROUND_TRUTH:
            sensor = SensorFactory.CreateGroundTruthSensor();
            genericOptionsGroup.gameObject.SetActive(true);
            gnssInsOptionsGroup.gameObject.SetActive(false);
            raptorOptionsGroup.gameObject.SetActive(false);
            break;
        }

        updateSensorObj();
    }

    private void chosenLinkNameChanged(TMP_Dropdown dropdown)
    {
        int idx = dropdown.value;
        // consider the case where no link or urdf exists (none in dropdown, custom link in options)
        
        updateSensorObj();
    }

    private void chosenSensorModelChanged(TMP_Dropdown dropdown)
    {
        int idx = dropdown.value;

        updateSensorObj();
    }

    private void namespaceChanged(TMP_InputField inputField)
    {
        string inputText = inputField.text;

        updateSensorObj();
    }

    private void optionsButtonPressed()
    {
        if (optionsPanel.gameObject.activeSelf)
        {
            optionsPanel.gameObject.SetActive(false);
        }
        else
        {
            optionsPanel.gameObject.SetActive(true);
        }
    }

    private void deleteButtonPressed()
    {
        if (sensorMenu.prefabList.Count <= 1)
        {
            return;
        }
        else
        {
            OnSensorDeleted?.Invoke(this);
            Destroy(this.gameObject);
        }
    }

    private void updateSensorObj()
    {
        TMP_InputField tx = optionsPanel.transform.Find("TXInputField").GetComponent<TMP_InputField>();
        TMP_InputField ty = optionsPanel.transform.Find("TYInputField").GetComponent<TMP_InputField>();
        TMP_InputField tz = optionsPanel.transform.Find("TZInputField").GetComponent<TMP_InputField>();
        TMP_InputField rx = optionsPanel.transform.Find("RXInputField").GetComponent<TMP_InputField>();
        TMP_InputField ry = optionsPanel.transform.Find("RYInputField").GetComponent<TMP_InputField>();
        TMP_InputField rz = optionsPanel.transform.Find("RZInputField").GetComponent<TMP_InputField>();

        sensor.IsActive = enabledToggle.isOn;
        sensor.SensorType = (SensorType)sensorTypeDropdown.value;
        sensor.SensorLink = findURDFLink(linkNameDropdown.value);
        sensor.TopicNamespace = namespaceInputField.text;
        sensor.Translation = new Vector3 (isFFieldValid(tx), isFFieldValid(ty), isFFieldValid(tz));
        sensor.Rotation = new Vector3 (isFFieldValid(rx), isFFieldValid(ry), isFFieldValid(rz));

        switch (sensor.SensorType)
        {
            case SensorType.GNSS_INS:
                updateGnssInsSensorOptions();
                break;
            
            case SensorType.RAPTOR:
                updateRaptorSensorOptions();
                break;

            case SensorType.LIDAR:
                updateLidarSensorOptions();
                break;
            
            case SensorType.CAMERA:
                updateCameraSensorOptions();
                break;
            
            case SensorType.RADAR:
                updateRadarSensorOptions();
                break;
            
            case SensorType.GROUND_TRUTH:
                updateGroundTruthSensorOptions();
                break;
        }

    }

    private void updateGnssInsSensorOptions()
    {
        GnssInsSensorOptions gnssOptions = sensor.Options as GnssInsSensorOptions;

        if (gnssOptions != null)
        {
            TMP_InputField posStdev = optionsPanel.transform.Find("GnssInsOptionsGroup/PositionStdevInput").GetComponent<TMP_InputField>();
            TMP_InputField headStdev = optionsPanel.transform.Find("GnssInsOptionsGroup/HeadingStdevInput").GetComponent<TMP_InputField>();
            TMP_InputField accCov = optionsPanel.transform.Find("GnssInsOptionsGroup/AccCovInput").GetComponent<TMP_InputField>();
            TMP_InputField gyroCov = optionsPanel.transform.Find("GnssInsOptionsGroup/GyroCovInput").GetComponent<TMP_InputField>();

            gnssOptions.Model = (SensorModel.GnssIns)sensorModelDropdown.value;
            gnssOptions.LatStdDev = isFFieldValid(posStdev);
            gnssOptions.LonStdDev = isFFieldValid(posStdev);
            gnssOptions.HgtStdDev = isFFieldValid(posStdev);
            gnssOptions.HeadingStdDev = isFFieldValid(headStdev);
            gnssOptions.AccelCovariance = isFFieldValid(accCov);
            gnssOptions.GyroCovariance = isFFieldValid(gyroCov);
        }
    }

    private void updateRaptorSensorOptions()
    {
        RaptorSensorOptions raptorOptions = sensor.Options as RaptorSensorOptions;

        if (raptorOptions != null)
        {
            Toggle hot = optionsPanel.transform.Find("RaptorOptionsGroup/HotStartToggle").GetComponent<Toggle>();
            Toggle can = optionsPanel.transform.Find("RaptorOptionsGroup/CanModeToggle").GetComponent<Toggle>();
            TMP_InputField outTopic = optionsPanel.transform.Find("RaptorOptionsGroup/CanOutTopicInput").GetComponent<TMP_InputField>();
            TMP_InputField inTopic = optionsPanel.transform.Find("RaptorOptionsGroup/CanInTopicInput").GetComponent<TMP_InputField>();

            raptorOptions.Model = (SensorModel.Generic)sensorModelDropdown.value;
            raptorOptions.IsHotStart = hot.isOn;
            raptorOptions.CanMode = can.isOn;
            raptorOptions.CanOutTopic = outTopic.text;
            raptorOptions.CanInTopic = inTopic.text;
        }
    }

    private void updateLidarSensorOptions()
    {
        LidarSensorOptions lidarOptions = sensor.Options as LidarSensorOptions;

        if (lidarOptions != null)
        {
            lidarOptions.Model = (SensorModel.Lidar)sensorModelDropdown.value;
        }
    }

    private void updateCameraSensorOptions() {}

    private void updateRadarSensorOptions() {}

    private void updateGroundTruthSensorOptions() {}

    private string findURDFLink(int linkIdx)
    {
        // TODO from dropdown, find link name
        return "finish this method";
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