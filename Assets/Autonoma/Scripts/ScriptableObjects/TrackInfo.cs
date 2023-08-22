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

[CreateAssetMenu]
public class TrackInfo : ScriptableObject
{
    // Start is called before the first frame update
    public List<double> innerX = new List<double>();
    public List<double> innerZ = new List<double>(); 
    public List<double> innerY = new List<double>(); 
    public List<double> heading = new List<double>();
    public List<double> heightInner = new List<double>();
    public List<double> bankingInner = new List<double>();
    public List<double> heightOuter = new List<double>();
    public List<double> bankingOuter = new List<double>();

}