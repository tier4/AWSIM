// Copyright 2019-2021 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using ROS2;

using addTwoIntsReq = example_interfaces.srv.AddTwoInts_Request;
using addTwoIntsResp = example_interfaces.srv.AddTwoInts_Response;

/// <summary>
/// An example class provided for testing of basic ROS2 client
/// </summary>
public class ROS2ClientExample : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IClient<addTwoIntsReq, addTwoIntsResp> addTwoIntsClient;
    private bool isRunning = false;
    private Task<addTwoIntsResp> asyncTask;

    IEnumerator periodicAsyncCall()
    {
        while (ros2Unity.Ok())
        {

            while (!addTwoIntsClient.IsServiceAvailable())
            {
                yield return new WaitForSecondsRealtime(1);
            }

            addTwoIntsReq request = new addTwoIntsReq();
            request.A = Random.Range(0, 100);
            request.B = Random.Range(0, 100);
            
            asyncTask = addTwoIntsClient.CallAsync(request);
            asyncTask.ContinueWith((task) => { Debug.Log("Got async answer " + task.Result.Sum); });
            
            yield return new WaitForSecondsRealtime(1);
        }
    }

    IEnumerator periodicCall()
    {
        while (ros2Unity.Ok())
        {

            while (!addTwoIntsClient.IsServiceAvailable())
            {
                yield return new WaitForSecondsRealtime(1);
            }

            addTwoIntsReq request = new addTwoIntsReq();
            request.A = Random.Range(0, 100);
            request.B = Random.Range(0, 100);
            var response = addTwoIntsClient.Call(request);

            Debug.Log("Got sync answer " + response.Sum);

            yield return new WaitForSecondsRealtime(1);
        }
    }

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityClient");
                addTwoIntsClient = ros2Node.CreateClient<addTwoIntsReq, addTwoIntsResp>(
                    "add_two_ints");
            }
        }
    }

    void Update()
    {
        if (!isRunning)
        {
            isRunning = true;

            // Async calls
            StartCoroutine(periodicAsyncCall());

            // Sync calls
            StartCoroutine(periodicCall());
        }
    }
}
