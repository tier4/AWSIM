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
using TMPro;
using UnityEngine.UI;
using UnityEngine.Networking;
using System.Linq;

public class HUDBestTime : MonoBehaviour
{
    public TMP_Text bestTimeText; 
    public TMP_Text lastTimeText; 
    public TMP_Text currTimeText; 
    //public TMP_Text lapCount;
    public LapTimer lapTimer;
    void Start(){}

    // Update is called once per frame
    void Update()
    {
        //lapCount.text = lapTimer.laptimes.Count.ToString();
        if (lapTimer.laptimes.Count > 0)
        {
            bestTimeText.text = FormatTime(lapTimer.laptimes.Min());//.ToString("00.00");
            lastTimeText.text = FormatTime(lapTimer.laptimes.Last());//.ToString("00.00");
        }
        else
        {
            bestTimeText.text =FormatTime(0.0f);      
            lastTimeText.text =FormatTime(0.0f);       
        }
        currTimeText.text = FormatTime(lapTimer.currLaptime);//.ToString("00.00");
    }

    public string convertToMinSecMillisec(float timer)
    {
        int minutes = Mathf.FloorToInt(timer / 60F);
        int seconds = Mathf.FloorToInt(timer - minutes * 60);
        string niceTime = string.Format("{0:0}:{1:00}", minutes, seconds);
        return niceTime;
    }
    public string FormatTime( float time )
    {
        int minutes = (int) time / 60 ;
        int seconds = (int) time - 60 * minutes;
        int milliseconds = (int) (100 * (time - minutes * 60 - seconds));
        return string.Format("{0:00}:{1:00}:{2:00}", minutes, seconds, milliseconds );
    }
}
