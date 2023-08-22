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
using Autonoma;
using UnityEngine;
using TMPro;

public class SteeringWheelController : MonoBehaviour
{
    public HUDManager hud;
    public CarController carController;
    public GameObject[] leds;
    public TextMeshPro gearText;
    public TextMeshPro speedText;

    private float minRpm = 6500f;
    private float maxRpm = 7400f;
    void Update()
    {
        // Update wheel angle
        transform.localRotation = Quaternion.Euler(0f, -180f, 
        -carController.steerAngleApplied*carController.vehicleParams.steeringRatio);   

        // Update the gear and speed text
        gearText.text = hud.gearText.text;
        speedText.text = hud.speedometerText.text;

        // Calculate which LED should be the last one turned on
        float rpmRange = maxRpm - minRpm;
        float normalizedRpm = (hud.hudRpm - minRpm) / rpmRange;
        int lastLedOn = Mathf.Clamp(Mathf.FloorToInt(normalizedRpm * leds.Length), 0, leds.Length - 1);

        // Turn the LEDs on/off based on the current RPM
        for (int i = 0; i < leds.Length; i++)
        {
            leds[i].SetActive(i <= lastLedOn);
        }
    }
}
