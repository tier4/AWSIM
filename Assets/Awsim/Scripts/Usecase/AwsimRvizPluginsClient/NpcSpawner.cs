// Copyright 2025 TIER IV, Inc.
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

using System;
using ROS2;
using UnityEngine;
using Awsim.Common;
using Awsim.Entity;
using System.Collections.Generic;
using Unity.VisualScripting;
using System.Linq;

namespace Awsim.Usecase.AwsimRvizPlugins
{
    /// <summary>
    /// Structure of vehicle prefab configuration
    /// </summary>
    [Serializable]
    public class NpcVehicleDictionaryContent
    {
        public string Name;
        public PoseVehicle Prefab;
    }

    /// <summary>
    /// Structure of pedestrian prefab configuration
    /// </summary>
    [Serializable]
    public class NpcPedestrianDictionaryContent
    {
        public string Name;
        public Pedestrian Prefab;
    }

    /// <summary>
    /// Structure of configuration of NpcSpawner
    /// </summary>
    [Serializable]
    public class NpcSpawnerSettings
    {
        [SerializeField] List<NpcVehicleDictionaryContent> _npcVehiclePrefabs;
        [SerializeField] List<NpcPedestrianDictionaryContent> _npcPedestrianPrefabs;
        [SerializeField] string _npcPositionTopic = "/awsim/awsim_rviz_plugins/npc_spawner/pose_with_covariance";
        [SerializeField] string _npcNameTopic = "/awsim/awsim_rviz_plugins/npc_spawner/npc_name";
        [SerializeField] string _npcVelocityTopic= "/awsim/awsim_rviz_plugins/npc_spawner/npc_velocity";
        [SerializeField] string _npcNameListTopic = "/awsim/awsim_rviz_plugins/npc_spawner/npc_name_list";

        public List<NpcVehicleDictionaryContent> NpcVehiclePrefabs { get => _npcVehiclePrefabs; private set => _npcVehiclePrefabs = value; }
        public List<NpcPedestrianDictionaryContent> NpcPedestrianPrefabs { get => _npcPedestrianPrefabs; private set => _npcPedestrianPrefabs = value; }
        public string NpcPositionTopic { get => _npcPositionTopic; private set => _npcPositionTopic = value; }
        public string NpcNameTopic { get => _npcNameTopic; private set => _npcNameTopic = value; }
        public string NpcVelocityTopic { get => _npcVelocityTopic; private set => _npcVelocityTopic = value; }
        public string NpcNameListTopic { get => _npcNameListTopic; private set => _npcNameListTopic = value; }
    }

    /// <summary>
    /// Feature of spawn npc by AwsimRvizPlugins
    /// </summary>
    public class NpcSpawner
    {
        // Subscribers.
        ISubscription<geometry_msgs.msg.PoseWithCovarianceStamped> _positionSubscriber;
        ISubscription<std_msgs.msg.String> _nameSubscriber;
        ISubscription<std_msgs.msg.Float32> _velocitySubscriber;
        IPublisher<std_msgs.msg.String> _nameListPublisher;

        List<PoseVehicle> _npcVehicleList = new List<PoseVehicle>();
        List<PoseVehicle> _readyVehicleList = new List<PoseVehicle>();
        List<Pedestrian> _npcPedestrianList = new List<Pedestrian>();

        Transform _parent;
        Dictionary<string, PoseVehicle> _npcVehiclePrefabDict = new Dictionary<string, PoseVehicle>();
        Dictionary<string, Pedestrian> _npcPedestrianPrefabDict = new Dictionary<string, Pedestrian>();

        Vector3 _position = Vector3.zero;
        Quaternion _rotation = new Quaternion();
        bool _spawnFlag = false;
        bool _npcVehicleDespawnedFlag = false;
        bool _npcPedestrianDespawnedFlag = false;

        // km/h -> m/frame (60fps)
        float _speed_converter = 1000 * Time.fixedDeltaTime / (60 * 60);
        float _velocity;
        string _spawnPrefabName;

        /// <summary>
        /// Load prefab from inspector
        /// Initialize parameter and ROS2 subscriber
        /// Publish npc name to AwsimRvizPlugins
        /// </summary>
        public NpcSpawner(Transform parent, NpcSpawnerSettings settings, QualityOfServiceProfile qos)
        {
            _velocity = 10.0f * _speed_converter;
            _parent = parent;

            foreach (NpcVehicleDictionaryContent prefab in settings.NpcVehiclePrefabs)
                _npcVehiclePrefabDict[prefab.Name] = prefab.Prefab;
            foreach (NpcPedestrianDictionaryContent prefab in settings.NpcPedestrianPrefabs)
                _npcPedestrianPrefabDict[prefab.Name] = prefab.Prefab;

            _spawnPrefabName = settings.NpcVehiclePrefabs.Count() != 0 ? settings.NpcVehiclePrefabs[0].Name : "";

            _positionSubscriber
                = AwsimRos2Node.CreateSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(
                    settings.NpcPositionTopic, msg =>
                    {
                        Vector3 subscribed_pos = new Vector3(
                            (float)msg.Pose.Pose.Position.X,
                            (float)msg.Pose.Pose.Position.Y,
                            (float)msg.Pose.Pose.Position.Z);

                        _position = Ros2Utility.Ros2ToUnityPosition(subscribed_pos - MgrsPosition.Instance.Mgrs.Position);
                        _rotation = Ros2Utility.Ros2ToUnityRotation(msg.Pose.Pose.Orientation);
                        _spawnFlag = true;
                    }, qos);

            qos.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            _nameSubscriber
                = AwsimRos2Node.CreateSubscription<std_msgs.msg.String>(
                    settings.NpcNameTopic, msg =>
                    {
                        _spawnPrefabName = msg.Data;
                    }, qos);

            _velocitySubscriber
                = AwsimRos2Node.CreateSubscription<std_msgs.msg.Float32>(
                    settings.NpcVelocityTopic, msg =>
                    {
                        _velocity = msg.Data * _speed_converter;
                    }, qos);

            _nameListPublisher = AwsimRos2Node.CreatePublisher<std_msgs.msg.String>(settings.NpcNameListTopic, qos);

            std_msgs.msg.String msg = new std_msgs.msg.String();
            msg.Data = String.Join(",", _npcVehiclePrefabDict.Keys.ToArray().Concat(_npcPedestrianPrefabDict.Keys.ToArray()));
            _nameListPublisher.Publish(msg);
        }

        public void OnUpdate()
        {
            foreach (PoseVehicle npc in _readyVehicleList)
            {
                if (npc != null)
                    npc.OnUpdate();
            }

            foreach (PoseVehicle npc in _npcVehicleList)
            {
                if (npc != null)
                    npc.OnUpdate();
            }

            foreach (Pedestrian npc in _npcPedestrianList)
            {
                if (npc != null)
                    npc.OnUpdate();
            }
        }

        /// <summary>
        /// Move position of spawned npc
        /// Spawn npc if _spawnFlag is true (i.e., coordinates of spawn is subscribed)
        /// </summary>
        public void OnFixedUpdate()
        {
            if (_npcVehicleDespawnedFlag == true)
            {
                _npcVehicleList.RemoveAll(n => n == null);
                _readyVehicleList.RemoveAll(n => n == null);
                _npcVehicleDespawnedFlag = false;
            }

            if (_npcPedestrianDespawnedFlag == true)
            {
                _npcPedestrianList.RemoveAll(n => n == null);
                _npcPedestrianDespawnedFlag = false;
            }

            bool anyVehicleGrounded = false;
            foreach (PoseVehicle npc in _readyVehicleList)
            {
                if (npc.IsGrounded)
                {
                    _npcVehicleList.Add(npc);
                    anyVehicleGrounded = true;
                }
                npc.OnFixedUpdate();
            }
            if (anyVehicleGrounded)
                _readyVehicleList.RemoveAll(n => n.IsGrounded);

            foreach (PoseVehicle npc in _npcVehicleList)
            {
                npc.PoseInput = new Pose(npc.transform.position + npc.transform.rotation * new Vector3(0, 0, _velocity), npc.transform.rotation);
                npc.OnFixedUpdate();
            }

            foreach (Pedestrian npc in _npcPedestrianList)
            {
                npc.PoseInput = new Pose(npc.transform.position + npc.transform.rotation * new Vector3(0, 0, _velocity), npc.transform.rotation);
                npc.OnFixedUpdate();
            }

            if (_spawnFlag)
                Spawn();
            _spawnFlag = false;
        }

        /// <summary>
        /// Destroy (remove) ROS2 subscriber
        /// </summary>
        public void OnDestroy()
        {
            AwsimRos2Node.RemoveSubscription<geometry_msgs.msg.PoseWithCovarianceStamped>(_positionSubscriber);
            AwsimRos2Node.RemoveSubscription<std_msgs.msg.String>(_nameSubscriber);
            AwsimRos2Node.RemoveSubscription<std_msgs.msg.Float32>(_velocitySubscriber);
            AwsimRos2Node.RemovePublisher<std_msgs.msg.String>(_nameListPublisher);
        }

        void Spawn()
        {
            if (_npcVehiclePrefabDict.ContainsKey(_spawnPrefabName))
            {
                PoseVehicle prefab = _npcVehiclePrefabDict[_spawnPrefabName];
                if (CalcSpawnCoordinate(_position, _rotation, prefab.Bounds.extents, out Vector3 position, out Quaternion rotation))
                {
                    PoseVehicle npc = PoseVehicle.Instantiate(prefab, position, rotation, _parent);
                    npc.Initialize();
                    npc.OnCollisionEnterAction += (Collision collision) =>
                        {
                            GameObject obj = npc.gameObject;
                            UnityObject.DestroySafe<GameObject>(ref obj);
                            _npcVehicleDespawnedFlag = true;
                        };
                    _readyVehicleList.Add(npc);
                }
                else
                {
                    Debug.LogWarning("No mesh or collider detected on target location. Please ensure that the target location is on a mesh or collider.");
                }
            }
            if (_npcPedestrianPrefabDict.ContainsKey(_spawnPrefabName))
            {
                Pedestrian prefab = _npcPedestrianPrefabDict[_spawnPrefabName];
                Vector3 extents = new Vector3(0.125f, 0.5f, 0.125f);
                if (CalcSpawnCoordinate(_position, _rotation, extents, out Vector3 position, out Quaternion rotation))
                {
                    Pedestrian npc = Pedestrian.Instantiate(prefab, position, rotation, _parent);
                    npc.Initialize();
                    npc.OnTriggerEnterAction += (Collider collider) =>
                        {
                            GameObject obj = npc.gameObject;
                            UnityObject.DestroySafe<GameObject>(ref obj);
                            _npcPedestrianDespawnedFlag = true;
                        };

                    BoxCollider collider = npc.AddComponent<BoxCollider>();
                    collider.center = new Vector3(0, 1, -0.1f);
                    collider.size = extents * 2;
                    collider.isTrigger = true;

                    _npcPedestrianList.Add(npc);
                }
                else
                {
                    Debug.LogWarning("No mesh or collider detected on target location. Please ensure that the target location is on a mesh or collider.");
                }
            }

            // --- inner methods ---
            static bool CalcSpawnCoordinate(Vector3 centerPosition, Quaternion inputRotation, Vector3 extents, out Vector3 destPosition, out Quaternion destRotation)
            {
                Vector3 rayDirection = Vector3.down;
                destPosition = new Vector3();
                destRotation = new Quaternion();

                Vector3 rayOriginForward = new Vector3(centerPosition.x, 1000.0f, centerPosition.z) + inputRotation * Vector3.forward * extents.z;
                if (!Physics.Raycast(rayOriginForward, rayDirection, out RaycastHit hitForward, Mathf.Infinity))
                    return false;

                Vector3 rayOriginBack = new Vector3(centerPosition.x, 1000.0f, centerPosition.z) + inputRotation * Vector3.back * extents.z;
                if (!Physics.Raycast(rayOriginBack, rayDirection, out RaycastHit hitBack, Mathf.Infinity))
                    return false;

                Vector3 rayOriginLeft = new Vector3(centerPosition.x, 1000.0f, centerPosition.z) + inputRotation * Vector3.left * extents.x;
                if (!Physics.Raycast(rayOriginLeft, rayDirection, out RaycastHit hitLeft, Mathf.Infinity))
                    return false;

                Vector3 rayOriginRight = new Vector3(centerPosition.x, 1000.0f, centerPosition.z) + inputRotation * Vector3.right * extents.x;
                if (!Physics.Raycast(rayOriginRight, rayDirection, out RaycastHit hitRight, Mathf.Infinity))
                    return false;

                float frontBackDist = Vector3.Distance(rayOriginBack, rayOriginForward);
                float frontBackYOffset = hitForward.point.y - hitBack.point.y;
                float pitch = Mathf.Rad2Deg * Mathf.Atan(frontBackYOffset / frontBackDist);

                float leftRightDist = Vector3.Distance(rayOriginRight, rayOriginLeft);
                float leftRightYOffset = hitRight.point.y - hitLeft.point.y;
                float roll = Mathf.Rad2Deg * Mathf.Atan(leftRightYOffset / leftRightDist);

                destPosition = (hitForward.point + hitBack.point) / 2 + new Vector3(0.0f, 1.0f, 0.0f);
                // NOTE: Unity uses a left-handed coordinate.
                destRotation = Quaternion.Euler(-pitch, inputRotation.eulerAngles.y, roll);
                return true;
            }
        }
    }
}