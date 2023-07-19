using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;

namespace AWSIM
{
    /// <summary>
    /// Convenience static class for command line.
    /// </summary>
    public static class CommandLineUtility
    {
        /// <summary>
        /// Get instance from Json by specifying Path.
        /// </summary>
        /// <typeparam name="T">Class to be converted from json.</typeparam>
        /// <param name="path">Path where Json is located.</param>
        /// <returns></returns>
        public static T LoadJsonFromPath<T>(string path)
        {
            try
            {
                string rawContent = File.ReadAllText(path);
                T content = JsonUtility.FromJson<T>(rawContent);
                return content;
            }
            catch (Exception exceptipn)
            {
                Debug.LogError(exceptipn.Message);
                return default;
            }
        }

        /// <summary>
        /// Get the corresponding command line argument.
        /// </summary>
        /// <param name="param">Parameter</param>
        /// <returns></returns>
        public static string GetCommandLineArg(string param)
        {
            var cmdArgs = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < cmdArgs.Length; i++)
            {
                if (cmdArgs[i] == param && cmdArgs.Length > i + 1)
                {
                    return cmdArgs[i + 1];
                }
            }

            return null;
        }

        /// <summary>
        /// Get the corresponding command line argument.
        /// </summary>
        /// <param name="arg">Argument</param>
        /// <param name="param">Parameter</param>
        /// <returns></returns>
        public static bool GetCommandLineArg(out string arg, string param)
        {
            var _arg = GetCommandLineArg(param);
            if (_arg != null)
            {
                arg = _arg;
                return true;
            }

            arg = "";
            return false;
        }
    }
}