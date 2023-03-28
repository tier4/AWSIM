using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

public class Fps_display : MonoBehaviour
{
    int frameCount;
    int Length;
    float prevTime;
    float Fps;
    float Median;
    float Average;

    List<float> FpsList = new List<float>();
    
    
    
    // Start is called before the first frame update
    void Start()
    {
	frameCount = 0;
	prevTime = 0.0f;
        
    }

    // Update is called once per frame
    void Update()
    {
        frameCount++;
        
        float time = Time.realtimeSinceStartup - prevTime;
        
        if (time >= 0.5f) {
            Fps = frameCount / time;
            
            FpsList.Add(Fps);
            
            frameCount = 0;
            prevTime = Time.realtimeSinceStartup;
        }
    }
    
    void OnApplicationQuit()
    {
    	FpsList.Sort();
    	Length = FpsList.Count;
    	
    	if (Length % 2 == 0) {
            	Median = FpsList[(Length - 1) / 2] + FpsList[(Length + 1) / 2];
    	}
    	else {
    		Median = FpsList[Length / 2];
    	}
    	
    	
    	Average = FpsList.Sum() / Length;
    	
	Debug.Log("Fps Median: " + Median);
        Debug.Log("Fps Average: " + Average);
    }
    
}
