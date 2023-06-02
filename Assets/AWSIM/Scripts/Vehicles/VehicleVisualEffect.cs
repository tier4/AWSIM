using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Set the VisualEffect of the vehicle.
    /// Includes steering, brake light, reverse light, and turn signal light.
    /// </summary>
    [RequireComponent(typeof(Vehicle))]
    public class VehicleVisualEffect : MonoBehaviour
    {
        [Serializable]
        public class EmissionMaterial
        {
            [SerializeField] MeshRenderer meshRenderer;
            [SerializeField] int materialIndex;
            [SerializeField] float lightingIntensity;
            [SerializeField] Color lightingColor;
            [SerializeField, Range(0, 1)] float lightingExposureWeight;

            Material material = null;
            Color defaultEmissiveColor;
            float defaultExposureWeight;
            bool isOn = false;

            const string EmissiveColor = "_EmissiveColor";
            const string EmissiveExposureWeight = "_EmissiveExposureWeight";

            public void Initialize()
            {
                if (material == null)
                {
                    material = meshRenderer.materials[materialIndex];
                    material.EnableKeyword("_EMISSION");
                    defaultEmissiveColor = material.GetColor(EmissiveColor);
                    defaultExposureWeight = material.GetFloat(EmissiveExposureWeight);
                }
            }

            public void Set(bool isLightOn)
            {
                if (this.isOn == isLightOn)
                    return;

                this.isOn = isLightOn;
                if (isLightOn)
                {
                    material.SetColor(EmissiveColor, lightingColor * lightingIntensity);
                    material.SetFloat(EmissiveExposureWeight, lightingExposureWeight);
                }
                else
                {
                    material.SetColor(EmissiveColor, defaultEmissiveColor);
                    material.SetFloat(EmissiveExposureWeight, defaultExposureWeight);
                }
            }

            public void Destroy()
            {
                if (material != null)
                    UnityEngine.Object.Destroy(material);
            }
        }

        [SerializeField] Vehicle vehicle;

        [Header("BrakeLight")]
        [SerializeField] EmissionMaterial[] brakeLights;

        [Header("TurnSignal")]
        [SerializeField] EmissionMaterial[] leftTurnSignalLights;
        [SerializeField] EmissionMaterial[] rightTurnSignalLights;
        [SerializeField] float turnSignalTimerIntervalSec = 0.5f;
        float turnSignalTimer = 0;
        bool turnSignalOn = false;

        [Header("ReverseLight")]
        [SerializeField] EmissionMaterial[] reverseLights;

        void Reset()
        {
            if (vehicle == null)
                vehicle = GetComponent<Vehicle>();
        }

        void Start()
        {
            foreach (var e in brakeLights) e.Initialize();
            foreach (var e in leftTurnSignalLights) e.Initialize();
            foreach (var e in rightTurnSignalLights) e.Initialize();
            foreach (var e in reverseLights) e.Initialize();
        }

        void Update()
        {
            // TODO: Implement steering object rotation.

            // brake light.
            var isBrakeLight = IsBrakeLight();
            ApplyLights(brakeLights, isBrakeLight);

            // reverse light.
            var isReverseLight = IsReverseLight();
            ApplyLights(reverseLights, isReverseLight);

            // turn signal light.
            if (IsTurnSignalOn() == false)
            {
                if (turnSignalTimer != 0)
                    turnSignalTimer = 0;

                if (turnSignalOn != false)
                    turnSignalOn = false;

                ApplyLights(leftTurnSignalLights, false);
                ApplyLights(rightTurnSignalLights, false);

                return;
            }

            turnSignalTimer -= Time.deltaTime;
            if (turnSignalTimer < 0f)
            {
                turnSignalTimer = turnSignalTimerIntervalSec;
                turnSignalOn = !turnSignalOn;
            }

            var isTurnLeftLight = IsTurnLeftLight();
            ApplyLights(leftTurnSignalLights, isTurnLeftLight);

            var isTurnRightLight = IsTurnRightLight();
            ApplyLights(rightTurnSignalLights, isTurnRightLight);

            // ----- internal methods ------

            bool IsBrakeLight()
            {
                return (vehicle.AutomaticShiftInput == Vehicle.Shift.DRIVE && vehicle.AccelerationInput < 0)
                    || (vehicle.AutomaticShiftInput == Vehicle.Shift.REVERSE && vehicle.AccelerationInput < 0);
            }

            bool IsReverseLight()
            {
                return vehicle.AutomaticShiftInput == Vehicle.Shift.REVERSE;
            }

            bool IsTurnSignalOn()
            {
                return ((vehicle.SignalInput == Vehicle.TurnSignal.LEFT)
                    || ((vehicle.SignalInput == Vehicle.TurnSignal.RIGHT)
                    || (vehicle.SignalInput == Vehicle.TurnSignal.HAZARD)));
            }

            bool IsTurnLeftLight()
            {
                return ((vehicle.SignalInput == Vehicle.TurnSignal.LEFT)
                    || (vehicle.SignalInput == Vehicle.TurnSignal.HAZARD))
                    && turnSignalOn;
            }

            bool IsTurnRightLight()
            {
                return ((vehicle.SignalInput == Vehicle.TurnSignal.RIGHT)
                    || (vehicle.SignalInput == Vehicle.TurnSignal.HAZARD))
                    && turnSignalOn;
            }

            void ApplyLights(EmissionMaterial[] emissionMaterials, bool isOn)
            {
                foreach(var e in emissionMaterials)
                {
                    e.Set(isOn);
                }
            }
        }

        void OnDestroy()
        {
            foreach (var e in brakeLights) e.Destroy();
            foreach (var e in leftTurnSignalLights) e.Destroy();
            foreach (var e in rightTurnSignalLights) e.Destroy();
            foreach (var e in reverseLights) e.Destroy();
        }
    }
}