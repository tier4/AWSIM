// Copyright 2019-2022 Robotec.ai.
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

#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEngine;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;

namespace ROS2
{

/// <summary>
/// An internal class responsible for installing ros2-for-unity metadata files 
/// </summary>
internal class PostInstall : IPostprocessBuildWithReport
{
    public int callbackOrder { get { return 0; } }
    public void OnPostprocessBuild(BuildReport report)
    {
        var r2fuMetadataName = "metadata_ros2_for_unity.xml";
        var r2csMetadataName = "metadata_ros2cs.xml";

        // FileUtil.CopyFileOrDirectory: All file separators should be forward ones "/".
        var r2fuMeta = ROS2ForUnity.GetRos2ForUnityPath() + "/" + r2fuMetadataName; 
        var r2csMeta = ROS2ForUnity.GetPluginPath() + "/" + r2csMetadataName;
        var outputDir = Directory.GetParent(report.summary.outputPath);
        var execFilename = Path.GetFileNameWithoutExtension(report.summary.outputPath);
        FileUtil.CopyFileOrDirectory(
            r2fuMeta, outputDir + "/" + execFilename + "_Data/" + r2fuMetadataName);
        if (EditorUserBuildSettings.activeBuildTarget == BuildTarget.StandaloneLinux64) {
            FileUtil.CopyFileOrDirectory(
                r2csMeta, outputDir + "/" + execFilename + "_Data/Plugins/" + r2csMetadataName);

            // Copy versioned libraries (Unity skips them)
            Regex soWithVersionReg = new Regex(@".*\.so(\.[0-9])+$");
            var versionedLibs = new List<String>(Directory.GetFiles(ROS2ForUnity.GetPluginPath()))
                                    .Where(path => soWithVersionReg.IsMatch(path))
                                    .ToList();
            foreach (var libPath in versionedLibs) {
                FileUtil.CopyFileOrDirectory(
                    libPath, outputDir + "/" + execFilename + "_Data/Plugins/" + Path.GetFileName(libPath));
            }
        } else {
            FileUtil.CopyFileOrDirectory(
                r2csMeta, outputDir + "/" + execFilename + "_Data/Plugins/x86_64/" + r2csMetadataName);
        }
    }

}

}
#endif
