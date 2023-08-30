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

public class GlobalCameraManager : MonoBehaviour
{
    public List<CarCameraPair> allCarCameraList = new List<CarCameraPair>();
    public bool changeCamera; 
    public  int counter;
    public bool isInitialized = false;

    void Awake() {}
    void Update()
    { 
        if (GameManager.Instance.Status != GameManager.SimulationState.DRIVE)
        {
            return;
        }
        
        if (!isInitialized && allCarCameraList.Count > 0 )
        {
            counter = 0;
            openCamera();
            isInitialized = true;
        }
        changeCamera = Input.GetKeyDown(KeyCode.C);
        
        if (changeCamera)
        {             
            openCamera();
        }
    }

    void openCamera()
    {
        if(allCarCameraList[counter].Camera != null)
            allCarCameraList[counter].Camera.SetActive(false);

        counter = (counter + 1) % allCarCameraList.Count;

        allCarCameraList[counter].Camera.SetActive(true);

        GameObject activeCar = allCarCameraList[counter].Car;

        foreach (var carCameraPair in allCarCameraList)
        {
            GameObject car = carCameraPair.Car;

            if (car == activeCar)
            {
                EnableHUD(car);
            }
            else
            {
                DisableHUD(car);
            }
        }
    }

    void EnableHUD(GameObject car)
    {
        car.transform.Find("HUD").gameObject.SetActive(true);
    }

    void DisableHUD(GameObject car)
    {
        car.transform.Find("HUD").gameObject.SetActive(false);
    }
}