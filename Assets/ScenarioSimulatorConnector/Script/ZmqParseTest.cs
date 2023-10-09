using UnityEngine;
using ZeroMQ;
using SimulationApiSchema;
using System;
using System.Threading;

public class ZmqParseTest : MonoBehaviour
{
    [SerializeField] private string initializeResponseAddress = "tcp://127.0.0.1:5555";

    Thread thread;


    // Start is called before the first frame update
    void Start()
    {
        thread = new Thread(() =>
        {
            using (ZContext context = new ZContext())
            using (ZSocket responseSocket = new ZSocket(context, ZSocketType.REP))
            {
                responseSocket.Bind(initializeResponseAddress);

                ZPollItem pollItem = ZPollItem.CreateReceiver();

                ZError zError;
                ZMessage zMessage;

                while (true)
                {
                    if (responseSocket.PollIn(pollItem, out zMessage, out zError))
                    {
                        // message received
                        if (zMessage != null)
                        {
                            // get received message to buffer and get the size of request
                            byte[] buffer = new byte[1024];
                            int requestSize = zMessage.PopBytes(buffer, 0, buffer.Length);

                            // select bytes related to request message
                            byte[] requestBytes = new byte[requestSize];
                            Array.ConstrainedCopy(buffer, 0, requestBytes, 0, requestSize);

                            InitializeRequest request = InitializeRequest.Parser.ParseFrom(requestBytes);

                            Debug.Log(request.RealtimeFactor);
                            Debug.Log(request.StepTime);

                            responseSocket.SendFrame(new ZFrame("ok"));
                        }
                    }
                    else
                    {
                        if (zError == ZError.ETERM)
                        {
                            Debug.LogError("[ZMQ ERROR] ZMQ context was terminated.");
                            break;
                        }
                        else
                        {
                            Debug.LogWarning("[ZMQ ERROR] " + zError);
                        }
                    }
                }
            }
        });
        thread.Start();
    }


    private void OnDestroy()
    {
        if(thread != null)
        {
            thread.Abort();
            thread.Join();
            thread = null;
        }

    }

    private void OnApplicationQuit()
    {
        if(thread != null)
        {
            thread.Abort();
            thread.Join();
            thread = null;
        }
    }
}
