using System;
using System.Threading;
using UnityEngine;
using ZeroMQ;
using SimulationApiSchema;
using Google.Protobuf;

namespace AWSIM
{
    /// <summary>
    /// Class manages initialization of response server base on ZMQ library and handles incoming client requests.
    /// </summary>
    [RequireComponent(typeof(ScenarioSimulatorRequestProcessor))]
    public class ScenarioSimulatorConnector : MonoBehaviour
    {
        #region [Variables]

        [Header("Address")]
        [SerializeField] private string serverResponseAdress = "tcp://127.0.0.1:8080";

        private ScenarioSimulatorRequestProcessor requestProcessor = default;
        private Thread thread = default;

        #endregion

        #region [Life Cycle - Unity Messages]

        private void Awake()
        {
            CollectComponents();
        }

        private void CollectComponents()
        {
            requestProcessor = this.gameObject.GetComponent<ScenarioSimulatorRequestProcessor>();
        }

        private void Start()
        {
            Initialize();
            StartServerResponseThread();
        }

        private void Initialize()
        {       
            requestProcessor.Initialize();
        }


        private void OnDestroy()
        {
            StopServerResponseThread();
            Dispose();
        }

        private void OnApplicationQuit()
        {
            StopServerResponseThread();
            Dispose();
        }


        private void Dispose()
        {
            requestProcessor.Dispose();
        }

        #endregion

        #region [Life Cycle - Thread]

        private void StartServerResponseThread()
        {            
            thread = ServerResponseThread();
        }

        private void StopServerResponseThread()
        {
            if(thread != null)
            {
                thread.Abort();
                thread = null;
            }
        }

        #endregion

        #region [Private Methods]

        private Thread ServerResponseThread()
        {
            Thread thread = new Thread(() =>
            {
                using (ZContext context = new ZContext())
                {
                    using (ZSocket responseSocket = new ZSocket(context, ZSocketType.REP))
                    {            
                        responseSocket.Bind(serverResponseAdress);

                        ZPollItem pollItem = ZPollItem.CreateReceiver();
                        ZError zError;
                        ZMessage zMessage;

                        while(true)
                        {
                            // pull message when received or wait 10 sec and check for errors
                            if (responseSocket.PollIn(pollItem, out zMessage, out zError))
                            {
                                // message received
                                if (zMessage != null)
                                {
                                    // get received message to buffer and get the size of request
                                    for (int i = 0; i < zMessage.Count; i++) {
                                        byte[] buffer = new byte[zMessage[i].Length];
                                        long requestSize = zMessage.PopBytes(buffer, 0, buffer.Length);

                                        SimulationRequest request = SimulationRequest.Parser.ParseFrom(buffer);
                                        SimulationResponse response = requestProcessor.Process(request);

                                        byte[] responseBytes = response.ToByteArray();
                                        if (!responseSocket.SendBytes(responseBytes, 0, responseBytes.Length))
                                        {
                                            Debug.LogWarning("[ZMQ ERROR] Failed to send a response.");
                                        }
                                    }
                                }
                            }
                            else
                            {
                                if (zError == ZError.ETERM)
                                {
                                    Debug.LogError("[ZMQ ERROR] ZMQ context was terminated.");
                                }
                                else
                                {
                                    Debug.LogWarning("[ZMQ ERROR] " + zError);
                                }
                            }
                        }                       
                   }
                }
            });
            thread.Name = "ServerResponse";
            thread.Start();
            return thread;
        }

        #endregion
    }
}
