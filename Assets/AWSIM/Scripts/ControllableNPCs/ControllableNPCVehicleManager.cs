using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Manager class for controllable NPC vehicle functionality.
    /// </summary>
    public class ControllableNPCVehicleManager : MonoBehaviour
    {
        [System. Serializable]
        public class ControllableNPCPrefabHandler
        {
            public string Name;
            public GameObject Prefab;
        }

        [Header("EGO Vehicle")]
        [SerializeField] private Vehicle egoVehicle = default;

        [Header("Components")]
        [SerializeField] private Transform spawnPoint = default;
        [SerializeField] private ControllableNPCVehicleCamera npcVehicleCamera = default;

        [Header("UI")]
        [SerializeField] private ControllableNPCVehicleUI uiView = default; 

        [Header("NPC Prefabs")]
        [SerializeField] private List<ControllableNPCPrefabHandler> controllableNPCPrefabs = default;

        [Header("Controlled NPC")]
        [SerializeField] private GameObject npcVehiclePrefab = default;

        private Vehicle controlledNPC = default;
        private ControllableNPCVehicleOverrideInputManager controlledNPCInputManager = default;

        private bool isInitialized = false;

        #region [Init]

        private void Start()
        {
            if(!isInitialized)
            {
                Initialize(npcVehiclePrefab, spawnPoint.position, spawnPoint.rotation);
            }
        }

        /// <summary>
        /// When invoked before the Unity Start message, it initializes the controllable manager with the selected NPC vehicle at a specific position and rotation.
        /// </summary>
        public void DoStart(string npcName, Vector3 npcPosition, Quaternion npcRotation)
        {
            if(!isInitialized)
            {
                ControllableNPCPrefabHandler npcPrefabHandler = controllableNPCPrefabs.FirstOrDefault(e => string.CompareOrdinal(e.Name, npcName) == 0);
                if(npcPrefabHandler == null)
                {
                    return;
                }

                Initialize(npcPrefabHandler.Prefab, npcPosition, npcRotation);
            }
        }

        private void Initialize(GameObject npcPrefab, Vector3 npcPosition, Quaternion npcRotation)
        {
            SpawnNPC(npcPrefab, npcPosition, npcRotation);
            InitCamera();
            InitUI();
            DisableEGOManualInput();
            isInitialized = true;
        }

        #endregion

        #region [Listeners]

        private void OnEnable() 
        {
            uiView.onVehicleDeviceInputChanged += OnVehicleDeviceInputChanged;
        }

        private void OnDisable()
        {
            uiView.onVehicleDeviceInputChanged -= OnVehicleDeviceInputChanged;
        }

        /// <summary>
        /// Listener of the UI event dispatched on changed the manually input device type.
        /// </summary>
        private void OnVehicleDeviceInputChanged(ControllableNPCVehicleOverrideInputManager.VehicleDeviceInput inputType)
        {
            ControllableNPCVehicleOverrideInputManager.VehicleDeviceInputMessageHandler messageHandler = null;

            controlledNPCInputManager.SetManuallyInput(inputType, out messageHandler);
            if(messageHandler == null)
            {
                uiView.SetDebugInfo(false, "", ControllableNPCVehicleOverrideInputManager.VehicleDeviceInputMessageHandler.MessageType.INFO);
            }
            else
            {
                uiView.SetDebugInfo(true, messageHandler.Message, messageHandler.Type);
            }

        }

        #endregion

        #region [Private Methods]

        /// <summary>
        /// Spawn NPC and assign it to controlled NPC.
        /// </summary>
        private void SpawnNPC(GameObject npcPrefab, Vector3 npcPosition, Quaternion npcRotation)
        {
            if(controlledNPC == null)
            {
                GameObject go = GameObject.Instantiate(npcPrefab, npcPosition, npcRotation, this.transform);
                controlledNPC = go?.GetComponent<Vehicle>();
                controlledNPCInputManager = go?.GetComponentInChildren<ControllableNPCVehicleOverrideInputManager>();
            }
        }

        /// <summary>
        /// Initialize camera view for controlled NPC.
        /// </summary>
        private void InitCamera()
        {
            if(controlledNPC == null)
            {
                return;
            }

            npcVehicleCamera.Init();
            npcVehicleCamera.SetFollowVehicle(controlledNPC);
        }

        /// <summary>
        /// Initialize UI view for controlled NPC.
        /// </summary>
        private void InitUI()
        {
            uiView.SetRenderTexture(npcVehicleCamera.RenderTexture);
            uiView.SetVehicle(controlledNPC);
            uiView.SetVehicleInputType(ControllableNPCVehicleOverrideInputManager.VehicleDeviceInput.KEYBOARD); //todo: read this value from input manager
        }

        /// <summary>
        /// Disable the possibility to switch to the manually input device for EGO.
        /// </summary>
        private void DisableEGOManualInput()
        {
            if(egoVehicle != null)
            {
                VehicleOverrideInputManager egoInputManager = egoVehicle.GetComponentInChildren<VehicleOverrideInputManager>();
                egoInputManager.ManuallyInputEnable = false;
            }
        }

        #endregion

    }

}

