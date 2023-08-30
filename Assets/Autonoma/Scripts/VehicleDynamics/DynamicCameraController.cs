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

using UnityEngine;
using VehicleDynamics;

public class DynamicCameraController : MonoBehaviour
{
    // Start is called before the first frame update

    public Transform car;
    public CarController carController;

    public bool useDynamicCamera = true;

    float beta, betaPrev;
         
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (useDynamicCamera)
        {
            float Vx = Mathf.Clamp(carController.V.x, 1f, carController.V.x);

            beta = Mathf.Atan(carController.V.y / Vx);

            beta = Vx > 1 ? beta : 0;
            beta = HelperFunctions.lowPassFirstOrder(beta,betaPrev,4.0f);

            float dist = 5f + carController.V.x * 0.04f;

            //dist = Mathf.Clamp(dist,5f,10f);
            transform.localPosition = new Vector3(dist * Mathf.Sin(beta), 1.68f, -dist * Mathf.Cos(beta));
            transform.localEulerAngles = new Vector3(-car.transform.eulerAngles.x, -beta * 180f / Mathf.PI, -car.transform.eulerAngles.z);
            //transform.localRotation =  new (car.transform.rotation.x, 0f, car.transform.rotation.y);    
            betaPrev = beta;
        }
    }
}
