/* 
Copyright 2022 Autonoma, Inc.

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
using UnityEngine.SceneManagement;
using UnityEngine.Events;

public class GameManager : MonoBehaviour
{
    public static GameManager Instance {get; private set;}
    public SettingsManager Settings {get; private set;}
    public BaseUIManager UIManager {get; private set;}
    
    public enum SimulationState
    {
        MENU = 0,
        LOADING = 1,
        DRIVE = 2,
        PAUSE = 3,
        RESTART = 4,
    }

    public SimulationState Status {get ; private set; }
    public string CurrentScene {get; private set;}

    private void Awake()
    {
        if (Instance != null && Instance != this) 
        { 
            Destroy(this); 
            return;
        } 
        else 
        { 
            Instance = this; 
        }

        Settings = GetComponentInChildren<SettingsManager>();
        return;
    }
    
    private void Start()
    {
        Status = SimulationState.MENU;
    }

    private void OnEnable()
    {
        SceneManager.sceneLoaded += OnSceneLoaded;
    }

    private void OnDisable()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
    }

    private void Update()
    {      
        if( Status == SimulationState.PAUSE )
        {
            Time.timeScale = 0;
        }
        else
        {
            Time.timeScale = 1;
        }

        if( Status == SimulationState.RESTART )
        {
            StartCoroutine(ChangeScene("DrivingScene"));
            Status = SimulationState.DRIVE;
        } 
    }

    public IEnumerator ChangeScene(string newScene)
    {
        CurrentScene = newScene;
        AsyncOperation loadingOperation = SceneManager.LoadSceneAsync(newScene, LoadSceneMode.Single);
        while (!loadingOperation.isDone)
        {
            yield return null;
        }
    }

    public void ChangeStateTo(SimulationState newStatus)
    {
        Status = newStatus;
    }

    private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        if (Status == SimulationState.MENU)
        {
            UIManager = FindObjectOfType<MenuUIManager>();
        }
        else
        {
            UIManager = FindObjectOfType<SimUIManager>();
        }
    }
} 
