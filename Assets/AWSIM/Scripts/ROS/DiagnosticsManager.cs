using System;
using System.IO;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using ROS2;
using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace AWSIM
{
    /// <summary>
    /// Prints diagnostics for desired elements described in "diagnostics.yaml" config file.
    ///
    /// Manager looks for config file in "Assets" directory while working in Editor, and
    /// in "<BINARY_NAME>_Data" when running released binary.
    ///
    /// Example contents of "diagnostics.yaml" file:
    ///
    ///     diagnostics:
    ///         "topic_state_monitor_initialpose3d: localization_topic_status":
    ///             alias: "Initialpose 3D status"
    ///             level: 0
    ///         "topic_state_monitor_pose_twist_fusion_filter_pose: localization_topic_status":
    ///             alias: "Pose Twist fusion status"
    ///             level: 0
    ///         "concatenate_data: concat_status":
    ///             alias: "Concat status"
    ///             level: 0
    ///
    /// where:
    ///     alias - on screen substitution, if not set, the whole name is used,
    ///     level - log level, messages with lower level are filtered out.
    /// </summary>
    public class DiagnosticsManager : MonoBehaviour
    {
        public struct DiagnosticEntry
        {
            public int timestampSec;
            public uint timestampNsec;
            public byte level;
            public string name;
            public string message;
            public string hardwareId;
            public Dictionary<string, string> content;
        }

        public class DiagnosticConfigEntry
        {
            public byte level { get; set; }
            public string alias { get; set; }
        };

        public class DiagnosticsConfig
        {
            public Dictionary<string, DiagnosticConfigEntry> Diagnostics { get; set; }
        }

        [SerializeField] string topicName = "/diagnostics";
        [SerializeField] string configName = "diagnostics.yaml";
        Dictionary<string, DiagnosticEntry> diagnosticEntries = new Dictionary<string, DiagnosticEntry>();
        Dictionary<string, DiagnosticConfigEntry> diagnosticConfigEntries;
        List<string> namesToWatch = new List<string>();
        ISubscription<diagnostic_msgs.msg.DiagnosticArray> diagnosticsSubscription;
        ConcurrentQueue<DiagnosticEntry> concurrentQueue = new ConcurrentQueue<DiagnosticEntry>();
        DateTime epochTime = new DateTime(1970, 1, 1);
        public Text contentTextWindow;
        public Canvas diagnosticsCanvas;
        public GameObject scrollView;
        public GameObject clearButton;
        public Text toggleVisibilityText;

        void Start()
        {
            if(!ReadConfig()) {
                this.gameObject.SetActive(false);
                return;
            }

            diagnosticsSubscription
                = SimulatorROS2Node.CreateSubscription<diagnostic_msgs.msg.DiagnosticArray>(topicName, DiagnosticsCallback);
        }

        public void ToggleVisibility()
        {
            scrollView.SetActive(!scrollView.activeInHierarchy);
            clearButton.SetActive(!clearButton.activeInHierarchy);
            if (scrollView.activeInHierarchy)
            {
                toggleVisibilityText.text = "Hide diagnostics";
            } else {
                toggleVisibilityText.text = "Show diagnostics";
            }
        }

        public void Clear()
        {
            diagnosticEntries.Clear();
        }

        bool ReadConfig()
        {
            char separator = Path.DirectorySeparatorChar;
            string appPath = Application.dataPath;
            string configPath = appPath + separator + configName;
            if (File.Exists(configPath))
            {
                diagnosticConfigEntries = new Dictionary<string, DiagnosticConfigEntry>();
                string content = File.ReadAllText(configPath);

                var deserializer = new DeserializerBuilder()
                    .WithNamingConvention(UnderscoredNamingConvention.Instance)
                    .Build();

                DiagnosticsConfig config;
                try
                {
                    config = deserializer.Deserialize<DiagnosticsConfig>(content);
                }
                catch (System.Exception exception)
                {
                    
                    Debug.LogError("Couldn't parse diagnostics config file. Details: " + exception.Message);
                    return false;
                }

                foreach (var entry in config.Diagnostics)
                {
                    namesToWatch.Add(entry.Key);
                    diagnosticConfigEntries.Add(entry.Key, entry.Value);
                    Debug.Log("Registered diagnostic watch for: \"" + entry.Key + "\" with alias: "
                        + "\"" + entry.Value.alias + "\" and log level: " + entry.Value.level);
                }

                if(!diagnosticsCanvas.gameObject.activeInHierarchy) diagnosticsCanvas.gameObject.SetActive(true);

                return true;
            } else {
                return false;
            }

        }

        void DiagnosticsCallback(diagnostic_msgs.msg.DiagnosticArray msg)
        {
            foreach (var status in msg.Status)
            {
                if (namesToWatch.Contains(status.Name))
                {
                    DiagnosticEntry entry = new DiagnosticEntry();
                    entry.name = status.Name;
                    entry.message = status.Message;
                    entry.level = status.Level;
                    entry.hardwareId = status.Hardware_id;
                    entry.timestampSec = msg.Header.Stamp.Sec;
                    entry.timestampNsec = msg.Header.Stamp.Nanosec;
                    if(entry.content != null) 
                    {
                        entry.content.Clear();
                    } else {
                        entry.content = new Dictionary<string, string>();
                    }
                    foreach (var value in status.Values)
                    {
                        entry.content.Add(value.Key, value.Value);
                    }
                    concurrentQueue.Enqueue(entry);
                }
            }
        }

        // Update is called once per frame
        void OnGUI()
        {
            var message = "";

            // Transfer diagnostic entries from ROS2 thread to main Unity thread
            DiagnosticEntry entryDequeued;
            if(concurrentQueue.TryDequeue(out entryDequeued))
            {
                diagnosticEntries[entryDequeued.name] = entryDequeued;
            }

            // Compose screen message
            bool firstSeparator = true;
            foreach (var entry in diagnosticEntries)
            {
                // Filter log level
                byte logLevelFromConfig = diagnosticConfigEntries[entry.Key].level;
                if (entry.Value.level < logLevelFromConfig)
                {
                    continue;
                }

                if (!firstSeparator) 
                {
                    message += "----------------------------" + System.Environment.NewLine;
                } else {
                    firstSeparator = false;
                }

                message += "<b>";

                // Handle levels
                switch (entry.Value.level)
                {
                    case (byte)0:
                        message += "[OK] ";
                        break;
                    case (byte)1:
                        message += "<color=yellow>[WARN]</color> ";
                        break;
                    case (byte)2:
                        message += "<color=red>[ERROR]</color> ";
                        break;
                    case (byte)3:
                        message += "<color=orange>[STALE]</color> ";
                        break;
                    default:
                        break;
                }

                // Handle timestamp
                epochTime = new DateTime(1970, 1, 1);
                DateTime result = epochTime.AddTicks(entry.Value.timestampSec);
                message += "[" + result.ToShortTimeString() + "] - ";

                // Handle displays
                string alias = diagnosticConfigEntries[entry.Key].alias;
                if (alias != null) {
                    message += alias + " " + System.Environment.NewLine;
                } else {
                    message += entry.Value.name + ": " + System.Environment.NewLine;
                }

                message += "</b>";
                message += "<i>Message:</i> " + entry.Value.message;
                message += System.Environment.NewLine;
            }

            contentTextWindow.text = message;
        }
    }
}
