using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Timers;

namespace AWSIM
{
    /// <summary>
    /// Traffic Light class.
    /// </summary>
    public class TrafficLight : MonoBehaviour
    {
        /// <summary>
        /// Type of each bulb.
        /// </summary>
        public enum BulbType
        {
            ANY_CIRCLE_BULB         = 0,
            RED_BULB         = 1,
            YELLOW_BULB      = 2,
            GREEN_BULB       = 3,
            LEFT_ARROW_BULB         = 4,
            RIGHT_ARROW_BULB        = 5,
            UP_ARROW_BULB           = 6,
            DOWN_ARROW_BULB         = 7,
            DOWN_LEFT_ARROW_BULB    = 8,
            DOWN_RIGHT_ARROW_BULB   = 9,
            CROSS_BULB              = 10,
        }

        /// <summary>
        /// Bulb lighting status.
        /// </summary>
        public enum BulbStatus
        {
            SOLID_OFF               = 0,        // Lights off.
            SOLID_ON                = 1,        // Lights on.
            FLASHING                = 2,        // Lights on every flashSec.
        }

        /// <summary>
        /// Bulb lighting color.
        /// </summary>
        public enum BulbColor
        {
            RED                     = 0,
            YELLOW                  = 1,
            GREEN                   = 2,
            WHITE                   = 3,
        }

        /// <summary>
        /// Used in TrafficLight.SetBulbData(). Based on the data in this class, the lighting of each bulb is controlled.
        /// </summary>
        [Serializable]
        public struct BulbData
        {
            public BulbType Type => type;

            public BulbColor Color => color;

            public BulbStatus Status => status;

            [SerializeField] BulbType type;
            [SerializeField] BulbColor color;
            [SerializeField] BulbStatus status;

            public BulbData(BulbType type, BulbColor color, BulbStatus status)
            {
                this.type = type;
                this.color = color;
                this.status = status;
            }
        }

        /// <summary>
        /// Define TrafficLight bulbs.
        /// </summary>
        [Serializable]
        class Bulb
        {
            /// <summary>
            /// Emission configuration to be applied to the material when the bulb is lit.
            /// </summary>
            [Serializable]
            public class EmissionConfig
            {
                public BulbColor BulbColor;
                public Color Color;
                public float Intensity;
                [Range(0, 1)] public float ExposureWeight;
            }

            public BulbType BulbType => bulbType;
            public BulbStatus BulbStatus => status;
            public BulbColor BulbColor => color;

            [SerializeField] BulbType bulbType;
            [SerializeField, Tooltip("Specifies the index of the material to be used for the bulb.")] 
            int materialIndex;

            // const parameters.
            const string EmissiveColor = "_EmissiveColor";
            const string EmissiveIntensity = "_EmissiveIntensity";
            const string EmissiveExposureWeight = "_EmissiveExposureWeight";
            const float flashIntervalSec = 0.5f;                // flash bulb lighting interval(sec).

            float timer = 0;                            // used for flashing status.     NOTE: Might as well make it static and refer to the same time. 
            Color defaultEmissiveColor;                 // default bulb material emissive color.
            float defaultEmissiveExposureWeight;        // default bulb mateiral emissive exposure weight
            Dictionary<BulbColor, EmissionConfig> bulbColorConfigPairs;
            Material material = null;                   // bulb mateiral(instance).
            bool initialized = false;
            BulbStatus status = BulbStatus.SOLID_OFF;
            BulbColor color;
            bool isLightOn = false;                     // used for flashing control.

            /// <summary>
            /// Called from TrafficLight class. Acquire and initialize bulb material.
            /// </summary>
            /// <param name="renderer">Renderer containing the bulb material.</param>
            /// <param name="bulbEmissionConfigs"></param>
            public void Initialize(Renderer renderer, EmissionConfig[] bulbEmissionConfigs)
            {
                // bulb color config.
                bulbColorConfigPairs = bulbEmissionConfigs.ToDictionary(x => x.BulbColor);

                // set material.
                material = renderer.materials[materialIndex];

                // cache default material parameters.
                defaultEmissiveColor = material.GetColor(EmissiveColor);
                defaultEmissiveExposureWeight = material.GetFloat(EmissiveExposureWeight);

                initialized = true;
            }

            /// <summary>
            /// Called from TrafficLight class.
            /// Set the bulb lighting.
            /// </summary>
            /// <param name="status">bulb status</param>
            /// <param name="color">bulb color</param>
            public void SetBulbLighting(BulbStatus status, BulbColor color)
            {
                if (initialized == false)
                {
                    Debug.LogError("Not initialized Traffic bulb.");
                    return;
                }

                if (this.status == status && this.color == color)
                    return;

                this.status = status;
                this.color = color;

                switch (status)
                {
                    case BulbStatus.SOLID_ON:
                        timer = 0;
                        Set(true); break;
                    case BulbStatus.SOLID_OFF:
                        timer = 0;
                        Set(false); break;
                    case BulbStatus.FLASHING:
                        timer = 0;
                        Set(true); break;
                }
            }

            /// <summary>
            /// Called from TrafficLight class. 
            /// Update timer for bulb flashing.
            /// </summary>
            /// <param name="deltaTime"></param>
            public void Update(float deltaTime)
            {
                if (status != BulbStatus.FLASHING)
                    return;

                timer += deltaTime;

                if (timer > flashIntervalSec)
                {
                    Set(!isLightOn);
                }
            }

            /// <summary>
            ///  Called from TrafficLight class. 
            ///  Discard the material instance.
            /// </summary>
            public void Destroy()
            {
                UnityEngine.Object.Destroy(material);
                initialized = false;
            }

            /// <summary>
            /// Change material parameters to control lighting.
            /// </summary>
            /// <param name="isLightOn">Light up bulb</param>
            void Set(bool isLightOn)
            {
                if (isLightOn)
                {
                    var config = bulbColorConfigPairs[color];
                    material.SetColor(EmissiveColor, config.Color * config.Intensity);
                    material.SetFloat(EmissiveExposureWeight, config.ExposureWeight);
                    this.isLightOn = true;
                    timer = 0;
                }
                else
                {
                    material.SetColor(EmissiveColor, defaultEmissiveColor);
                    material.SetFloat(EmissiveExposureWeight, defaultEmissiveExposureWeight);
                    this.isLightOn = false;
                    timer = 0;
                }
            }
        }

        [SerializeField, Tooltip("Set the Renderer containing the bulb material.")] 
        new Renderer renderer;

        /// <summary>
        /// Define the Emission parameter to be applied to the material when the Bulb is turned on.
        /// </summary>
        [Header("Bulb Emission config")]
        [SerializeField, Tooltip("Define the Emission parameter for BulbColor.")]
        Bulb.EmissionConfig[] bulbEmissionConfigs = new Bulb.EmissionConfig[]
        {
            new Bulb.EmissionConfig()
            {
                BulbColor = BulbColor.GREEN,
                Color = Color.green,
                Intensity = 14,
                ExposureWeight = 0.8f,
            },
            new Bulb.EmissionConfig()
            {
                BulbColor = BulbColor.YELLOW,
                Color = Color.yellow,
                Intensity = 14,
                ExposureWeight = 0.8f,
            },
            new Bulb.EmissionConfig()
            {
                BulbColor = BulbColor.RED,
                Color = Color.red,
                Intensity = 14,
                ExposureWeight = 0.8f,
            },
            new Bulb.EmissionConfig()
            {
                BulbColor = BulbColor.WHITE,
                Color = Color.white,
                Intensity = 14,
                ExposureWeight = 0.8f,
            },
        };

        [Header("Bulb material config"), Tooltip("Link the material of the bulb to the type.")]
        [SerializeField] Bulb[] bulbs;

        Dictionary<BulbType, Bulb> bulbPairs;
        int bulbCount;
        BulbData[] bulbDataArray;

        void Reset()
        {
            renderer = GetComponent<Renderer>();
        }

        void Awake()
        {
            bulbPairs = bulbs.ToDictionary(x => x.BulbType);
            bulbCount = bulbPairs.Count();
            bulbDataArray = new BulbData[bulbCount];

            // Initialize bulb materials.
            foreach (var e in bulbPairs.Values)
            {
                e.Initialize(renderer, bulbEmissionConfigs);
            }
        }

        void Update()
        {
            // Update timers for each Bulb.
            foreach (var e in bulbPairs.Values)
            {
                e.Update(Time.deltaTime);
            }
        }

        public void TurnOffAllBulbs()
        {
            foreach(var e in bulbs)
            {
                e.SetBulbLighting(TrafficLight.BulbStatus.SOLID_OFF, BulbColor.WHITE);
            }
        }

        /// <summary>
        /// Updates the status of each bulb of this traffic light.
        /// </summary>
        /// <param name="inputDatas">Input data to update each bulb.</param>
        public void SetBulbData(BulbData[] inputDatas)
        {
            for (int i = 0; i < inputDatas.Length; i++)
            {
                var inputData = inputDatas[i];
                SetBulbData(inputData);
            }
        }

        /// <summary>
        /// Updates the status of each bulb of this traffic light.
        /// </summary>
        /// <param name="inputData">Input data to update each bulb.</param>
        public void SetBulbData(BulbData inputData)
        {
            bulbPairs[inputData.Type].SetBulbLighting(inputData.Status, inputData.Color);
        }

        /// <summary>
        /// Get the current status of each bulb of the traffic light.
        /// </summary>
        /// <returns>bulb data array</returns>
        public BulbData[] GetBulbData()
        {
            int i = 0;

            foreach(var e in bulbPairs)
            {
                bulbDataArray[i] = new BulbData(e.Value.BulbType, e.Value.BulbColor, e.Value.BulbStatus);
                i++;
            }

            return bulbDataArray;
        }

        void OnDestroy()
        {
            // Destory bulb materials.
            foreach (var e in bulbs)
            {
                e.Destroy();
            }
        }
    }
}