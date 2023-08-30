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

public class MainMenuController : MonoBehaviour
{
    public Button CreateScenarioButton;
    public Button LicenseButton;
    public Button QuitButton;
    public Button SupportButton;
    public Button BackButton;
    public GameObject SupportPanel;
    void Start()
    {
        SupportPanel.gameObject.SetActive(false);
        CreateScenarioButton.onClick.AddListener( GameManager.Instance.UIManager.OnScenarioMenuPressed );
        LicenseButton.onClick.AddListener( GameManager.Instance.UIManager.OnLicenseMenuPressed );
        SupportButton.onClick.AddListener( OnSupportPressed );
        BackButton.onClick.AddListener( OnBackPressed );
        QuitButton.onClick.AddListener( GameManager.Instance.UIManager.OnQuitPressed );
    }

    void OnSupportPressed()
    {
        SupportPanel.gameObject.SetActive(true);
    }

    void OnBackPressed()
    {
        SupportPanel.gameObject.SetActive(false);
    }
}
