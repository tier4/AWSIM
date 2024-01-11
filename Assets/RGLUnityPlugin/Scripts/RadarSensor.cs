// Copyright 2023 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace RGLUnityPlugin
{
    public class RadarSensor : MonoBehaviour
    {
        /// <summary>
        /// Sensor processing and callbacks are automatically called in this hz.
        /// </summary>
        [FormerlySerializedAs("OutputHz")]
        [Range(0, 50)] public int automaticCaptureHz = 10;

        /// <summary>
        /// Delegate used in callbacks.
        /// </summary>
        public delegate void OnNewDataDelegate();

        /// <summary>
        /// Called when new data is generated via automatic capture.
        /// </summary>
        public OnNewDataDelegate onNewData;

        public bool applyDistanceGaussianNoise = true;
        public bool applyAngularGaussianNoise = true;

        public RadarModel modelPreset = RadarModel.SmartmicroDRVEGRD169MediumRange;

        // It is safer to refer to concrete property instead of using dictionary here because of static initialization order.
        public RadarConfiguration configuration = RadarConfigurationLibrary.SmartmicroDRVEGRD169MediumRange;

        private RGLNodeSequence rglGraphRadar;
        private RGLNodeSequence rglSubgraphToWorldFrame;
        private SceneManager sceneManager;

        private const string RadarRaysNodeId = "RADAR_RAYS";
        private const string RadarRangeNodeId = "RADAR_RANGE";
        private const string RadarPoseNodeId = "RADAR_POSE";
        private const string RadarRaytraceNodeId = "RADAR_RAYTRACE";
        private const string CompactNodeId = "COMPACT";
        private const string ToRadarFrameId = "TO_RADAR_FRAME";
        private const string ToWorldFrameId = "TO_WORLD_FRAME";
        private const string RemoveGroundNodeId = "REMOVE_GROUND";
        private const string RadarPostprocessNodeId = "RADAR_POSTPROCESS";
        private const string NoiseDistanceNodeId = "NOISE_DISTANCE";
        private const string NoiseRaysNodeId = "NOISE_RAYS";

        private RadarModel? validatedPreset = null;

        private Matrix4x4 lastTransform;
        private Matrix4x4 currentTransform;

        private float timer;
        private int fixedUpdatesInCurrentFrame = 0;
        private int lastUpdateFrame = -1;

        // Remove Ground Node parameters
        private const float GroundAngleThreshold = 5.0f;
        private const float GroundDistanceThreshold = 0.005f;
        private const float GroundFilterDistance = 0.5f;

        public void Awake()
        {
            rglGraphRadar = new RGLNodeSequence()
                .AddNodeRaysFromMat3x4f(RadarRaysNodeId, new Matrix4x4[1] { Matrix4x4.identity })
                .AddNodeRaysSetRange(RadarRangeNodeId, new Vector2[1] { new Vector2(0.0f, Mathf.Infinity) })
                .AddNodeRaysTransform(RadarPoseNodeId, Matrix4x4.identity)
                .AddNodeGaussianNoiseAngularRay(NoiseRaysNodeId, 0, 0)
                .AddNodeRaytrace(RadarRaytraceNodeId)
                .AddNodePointsCompact(CompactNodeId)
                .AddNodeGaussianNoiseDistance(NoiseDistanceNodeId, 0, 0, 0)
                .AddNodePointsTransform(ToRadarFrameId, Matrix4x4.identity)
                .AddNodePointsRemoveGround(RemoveGroundNodeId, GroundAngleThreshold * Mathf.Deg2Rad, GroundDistanceThreshold, GroundFilterDistance)
                .AddNodePointsRadarPostprocess(RadarPostprocessNodeId, 0.1f, 0.1f);

            rglSubgraphToWorldFrame = new RGLNodeSequence()
                .AddNodePointsTransform(ToWorldFrameId, Matrix4x4.identity);

            RGLNodeSequence.Connect(rglGraphRadar, rglSubgraphToWorldFrame);
        }

        public void Start()
        {
            sceneManager = FindObjectOfType<SceneManager>();
            if (sceneManager == null)
            {
                // TODO(prybicki): this is too tedious, implement automatic instantiation of RGL Scene Manager
                Debug.LogError($"RGL Scene Manager is not present on the scene. Destroying {name}.");
                Destroy(this);
                return;
            }
            OnValidate();

            // Apply initial transform of the sensor.
            lastTransform = gameObject.transform.localToWorldMatrix;
        }

        public void OnValidate()
        {
            // This tricky code ensures that configuring from a preset dropdown
            // in Unity Inspector works well in prefab edit mode and regular edit mode.
            bool presetChanged = validatedPreset != modelPreset;
            bool firstValidation = validatedPreset == null;
            if (!firstValidation && presetChanged)
            {
                configuration = RadarConfigurationLibrary.ByModel[modelPreset];
            }
            ApplyConfiguration(configuration);
            validatedPreset = modelPreset;
        }

        private void ApplyConfiguration(RadarConfiguration newConfig)
        {
            if (rglGraphRadar == null)
            {
                return;
            }

            rglGraphRadar.UpdateNodeRaysFromMat3x4f(RadarRaysNodeId, newConfig.GetRayPoses())
                .UpdateNodeRaysSetRange(RadarRangeNodeId, newConfig.GetRayRanges())
                .UpdateNodePointsRadarPostprocess(RadarPostprocessNodeId, newConfig.rangeSeparation, newConfig.azimuthSeparation * Mathf.Deg2Rad)
                .UpdateNodeGaussianNoiseAngularRay(NoiseRaysNodeId,
                    newConfig.noiseParams.angularNoiseMean * Mathf.Deg2Rad,
                    newConfig.noiseParams.angularNoiseStDev * Mathf.Deg2Rad)
                .UpdateNodeGaussianNoiseDistance(NoiseDistanceNodeId, newConfig.noiseParams.distanceNoiseMean,
                    newConfig.noiseParams.distanceNoiseStDevBase, newConfig.noiseParams.distanceNoiseStDevRisePerMeter);

            rglGraphRadar.SetActive(NoiseDistanceNodeId, applyDistanceGaussianNoise);
            rglGraphRadar.SetActive(NoiseRaysNodeId, applyAngularGaussianNoise);
        }

        // TODO(msz-rai): Radars should be triggered together with LiDARs to achieve the maximum performance.
        // TODO(msz-rai): See FixedUpdate method in LidarSensor component for more details.
        private void FixedUpdate()
        {
            if (lastUpdateFrame != Time.frameCount)
            {
                fixedUpdatesInCurrentFrame = 0;
                lastUpdateFrame = Time.frameCount;
            }
            fixedUpdatesInCurrentFrame += 1;

            if (automaticCaptureHz == 0.0f)
            {
                return;
            }

            timer += Time.deltaTime;

            // Update transforms
            lastTransform = currentTransform;
            currentTransform = gameObject.transform.localToWorldMatrix;

            var interval = 1.0f / automaticCaptureHz;
            if (timer + 0.00001f < interval)
                return;

            timer = 0;

            Capture();
        }

        public void ConnectToWorldFrame(RGLNodeSequence nodeSequence, bool compacted = true)
        {
            RGLNodeSequence.Connect(rglSubgraphToWorldFrame, nodeSequence);
        }

        public void ConnectToRadarFrame(RGLNodeSequence nodeSequence)
        {
            RGLNodeSequence.Connect(rglGraphRadar, nodeSequence);
        }

        public void Capture()
        {
            sceneManager.DoUpdate(fixedUpdatesInCurrentFrame);

            // Set radar pose
            Matrix4x4 radarPose = gameObject.transform.localToWorldMatrix;
            rglGraphRadar.UpdateNodeRaysTransform(RadarPoseNodeId, radarPose);
            rglGraphRadar.UpdateNodePointsTransform(ToRadarFrameId, radarPose.inverse);
            rglSubgraphToWorldFrame.UpdateNodePointsTransform(ToWorldFrameId, radarPose);

            SetVelocityToRaytrace();

            rglGraphRadar.Run();

            onNewData?.Invoke();
        }

        private void SetVelocityToRaytrace()
        {
            // Calculate delta transform of lidar.
            // Velocities must be in sensor-local coordinate frame.
            // Sensor linear velocity in m/s.
            Vector3 globalLinearVelocity = (currentTransform.GetColumn(3) - lastTransform.GetColumn(3)) / Time.deltaTime;
            Vector3 localLinearVelocity = gameObject.transform.InverseTransformDirection(globalLinearVelocity);

            Vector3 deltaRotation = Quaternion.LookRotation(currentTransform.GetColumn(2), currentTransform.GetColumn(1)).eulerAngles
                                    - Quaternion.LookRotation(lastTransform.GetColumn(2), lastTransform.GetColumn(1)).eulerAngles;
            // Fix delta rotation when switching between 0 and 360.
            deltaRotation = new Vector3(Mathf.DeltaAngle(0, deltaRotation.x), Mathf.DeltaAngle(0, deltaRotation.y), Mathf.DeltaAngle(0, deltaRotation.z));
            // Sensor angular velocity in rad/s.
            Vector3 localAngularVelocity = (deltaRotation * Mathf.Deg2Rad) / Time.deltaTime;

            rglGraphRadar.UpdateNodeRaytrace(RadarRaytraceNodeId, localLinearVelocity, localAngularVelocity, false);
        }
    }
}
