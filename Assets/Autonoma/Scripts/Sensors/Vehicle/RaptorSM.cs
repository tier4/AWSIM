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

public class RaptorSM : MonoBehaviour
{
	public int current_ct = 4;
	public int current_flag = 0;
	public int current_sys = 1;
	private bool hot_start = false;

	void Start()
	{
		hot_start = GameManager.Instance.Settings.myScenarioObj.HotStart;
	}
	void Update()
	{
		if(hot_start)
		{
			current_sys = 9;
		}
		else
		{
			transition();
		}
	}
	
	void transition()
	{
		switch(current_sys)
		{
			case 1: // PWR_ON
				current_sys = 2;
				break;
			case 2: // SUBSYS_CON
				current_sys = 3;
				break;
			case 3: // ACT_TESTING
				current_sys = 4;
				break;
			case 4: // ACT_TEST_DONE
				current_sys = 5;
				break;
			case 5: // CRANKREADY
				current_sys = 6;
				break;
			case 6: // PRECRANK_CHECK
				current_sys = 19;
				break;
			case 7: // CRANKING
				current_sys = 8;
				break;
			case 8: // ENG_RUNNING
				if(current_flag != 25) // Not Orange
				{
					current_sys = 9;
				}
				break;
			case 9: // DRIVING
				if(current_ct == 11) // Soft shutdown
				{
					current_sys = 19;
				}
				if(current_ct == 12) // Emergency shutdown
				{
					current_sys = 16;
				}
				if(current_flag == 33) // Engine kill
				{
					current_sys = 19;
				}
				break;
			case 16: // EMERGENCY SHUTDOWN
				current_sys = 19;
				break;
			case 19: // IGNITION
				if(current_ct == 5 && current_flag == 25) // Cranking and Orange
				{
					current_sys = 7;
				}
				break;
		}
	}
	void trigger_emergency()
	{
		current_sys = 16;
	}
	int get_sys_state()
	{
		return current_sys;
	}

} // end class

