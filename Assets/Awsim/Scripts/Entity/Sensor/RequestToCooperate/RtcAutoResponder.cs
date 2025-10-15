using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent; 
using UnityEngine;
using ROS2;

using CooperateStatusArray = tier4_rtc_msgs.msg.CooperateStatusArray;
using CooperateCommand = tier4_rtc_msgs.msg.CooperateCommand;
using CooperateStatus = tier4_rtc_msgs.msg.CooperateStatus;
using Command = tier4_rtc_msgs.msg.Command;
using CooperateCommands_Request = tier4_rtc_msgs.srv.CooperateCommands_Request;
using CooperateCommands_Response = tier4_rtc_msgs.srv.CooperateCommands_Response;
using Awsim.Common;


namespace Awsim.Entity
{
    /// <summary>
    /// Subscribes to Autoware cooperation status updates and automatically 
    /// sends cooperation requests (ACTIVATE commands) via service calls when conditions are met.
    /// </summary>
    public class RtcAutoResponder : MonoBehaviour
    {
        public float RtcStartDistanceThreshold
        {
            get => _rtcStartDistanceThreshold;
            set => _rtcStartDistanceThreshold = value;
        }
        public bool EnableRtcAutoResponder
        {
            get => _enableRtcAutoResponder;
            set => _enableRtcAutoResponder = value;
        }

        // ROS2 topics/services
        [SerializeField]
        string _rtcStatusTopic = "/api/external/get/rtc_status";
        [SerializeField]
        string _rtcCommandService = "/api/external/set/rtc_commands";
        ISubscription<CooperateStatusArray> _cooperateStatusSubscriber;
        IClient<CooperateCommands_Request, CooperateCommands_Response> _cooperateCommandsClient;

        // Config thresholds
        [SerializeField]
        float _egoStoppedVelocityThreshold = 0.1f;
        // NOTE: Some valid RTC statuses may have negative finish_distance
        // due to localization or numerical errors. This threshold (-20.0)
        // is based on TIER IV planning/control team advice.
        const float _finishDistanceThreshold = -20f;

        // Threshold for the 'start_distance' value in RTC status (meters).
        // When the 'start_distance' field received from Autoware is less than rtcStartDistanceThreshold
        // (and all other conditions are met), an ACTIVATE command can be sent.
        // Used to trigger a specifc RTC only when EGO is close enough to the relevant scene.
        [SerializeField]
        float _rtcStartDistanceThreshold = 10f;

        [SerializeField]
        Rigidbody _egoRigidbody;
        float _currentEgoVelocity;

        ConcurrentQueue<CooperateCommands_Request> _requests 
            = new ConcurrentQueue<CooperateCommands_Request>();

        bool _enableRtcAutoResponder = true;

        public void Initialize()
        {
            if (!_enableRtcAutoResponder)
            {
                Debug.LogWarning("[RtcAutoResponder] RTC Auto Responder feature is disabled by config.");
                return;
            }

            _cooperateStatusSubscriber = AwsimRos2Node.CreateSubscription<CooperateStatusArray>(
                _rtcStatusTopic,
                OnCooperateStatusArrayReceived
            );

            _cooperateCommandsClient = AwsimRos2Node.CreateClient<CooperateCommands_Request, CooperateCommands_Response>(
                _rtcCommandService
            );
        }

        public void OnFixedUpdate()
        {
            if (_egoRigidbody != null)
            {    
                _currentEgoVelocity = _egoRigidbody.linearVelocity.magnitude;
            }

            if (!_requests.IsEmpty && _requests.TryDequeue(out var request))
            {
                StartCoroutine(TrySendCooperateCommands(request));
            }
        }

        void OnCooperateStatusArrayReceived(CooperateStatusArray msg)
        {
            var commands = new List<CooperateCommand>();

            foreach (var status in msg.Statuses)
            {
                if (!ShouldSendCooperationRequest(status))
                {
                    continue;
                }

                commands.Add(new CooperateCommand
                {
                    Uuid    = status.Uuid,
                    Module  = status.Module,
                    Command = new Command { Type = Command.ACTIVATE }
                });
            }

            if (commands.Count == 0) return;

            var request = new CooperateCommands_Request
            {
                Stamp = msg.Stamp,
                Commands = commands.ToArray()
            };

            _requests.Enqueue(request);
        }

        bool ShouldSendCooperationRequest(CooperateStatus status)
        {
            bool isEgoStopped = _currentEgoVelocity < _egoStoppedVelocityThreshold;
            bool isWithinRequestDistance = status.Start_distance < _rtcStartDistanceThreshold;
            bool hasValidFinishDistance = status.Finish_distance > _finishDistanceThreshold;
            bool isAlreadyActive = status.Command_status.Type == Command.ACTIVATE;

            return !status.Auto_mode &&
                isEgoStopped &&
                isWithinRequestDistance &&
                !isAlreadyActive &&
                hasValidFinishDistance;
        }

        IEnumerator TrySendCooperateCommands(CooperateCommands_Request request, int attempts = 10, float intervalSeconds = 3f)
        {
            while (!_cooperateCommandsClient.IsServiceAvailable())
            {
                yield return new WaitForSecondsRealtime(intervalSeconds);
            }

            Exception lastEx = null;
            for (int attempt = 0; attempt < attempts; ++attempt)
            {
                var task = _cooperateCommandsClient.CallAsync(request);
                float elapsed = 0f;
                while (!task.IsCompleted && elapsed < intervalSeconds)
                {
                    yield return null;
                    elapsed += Time.deltaTime;
                }

                if (task.IsCompleted)
                {
                    if (task.Exception == null)
                    {
                        yield break;  // success
                    }
                    lastEx = task.Exception.InnerException ?? task.Exception;
                }
            }

            var baseMsg = $"[RtcAutoResponder]: RTC command failed after {attempts} attempts (~{attempts * intervalSeconds:F1}s).";
            if (lastEx != null)
            {
                Debug.LogError($"{baseMsg} Last error: {lastEx.Message}\n{lastEx.StackTrace}");
            }
            else
            {
                Debug.LogError(baseMsg);
            }
        }
    }
}
