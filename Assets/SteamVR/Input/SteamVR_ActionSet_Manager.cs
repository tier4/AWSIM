﻿//======= Copyright (c) Valve Corporation, All rights reserved. ===============

using UnityEngine;
using System.Collections;
using System;
using Valve.VR;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Text;

namespace Valve.VR
{
    /// <summary>
    /// Action sets are logical groupings of actions. Multiple sets can be active at one time.
    /// </summary>
    public static class SteamVR_ActionSet_Manager
    {
        public static VRActiveActionSet_t[] rawActiveActionSetArray
        {
            get
            {
                if (currentArraySize <= 0)
                    return null;
                else
                    return poolActiveActionSetArrays[currentArraySize];
            }
        }

        [NonSerialized]
        private static uint activeActionSetSize;

        [NonSerialized]
        private static bool changed = false;

        [NonSerialized]
        private static int currentArraySize;
        [NonSerialized]
        private static Dictionary<int, VRActiveActionSet_t[]> poolActiveActionSetArrays;

        public static void Initialize()
        {
            activeActionSetSize = (uint)(Marshal.SizeOf(typeof(VRActiveActionSet_t)));
            poolActiveActionSetArrays = new Dictionary<int, VRActiveActionSet_t[]>();
        }

        /// <summary>
        /// Disable all known action sets.
        /// </summary>
        public static void DisableAllActionSets()
        {
            for (int actionSetIndex = 0; actionSetIndex < SteamVR_Input.actionSets.Length; actionSetIndex++)
            {
                SteamVR_Input.actionSets[actionSetIndex].Deactivate(SteamVR_Input_Sources.Any);
                SteamVR_Input.actionSets[actionSetIndex].Deactivate(SteamVR_Input_Sources.LeftHand);
                SteamVR_Input.actionSets[actionSetIndex].Deactivate(SteamVR_Input_Sources.RightHand);
            }
        }

        private static int lastFrameUpdated;
        public static void UpdateActionStates(bool force = false)
        {
            if (force || Time.frameCount != lastFrameUpdated)
            {
                lastFrameUpdated = Time.frameCount;

                if (changed)
                {
                    UpdateActionSetsArray();
                }

                if (rawActiveActionSetArray != null && rawActiveActionSetArray.Length > 0)
                {
                    if (OpenVR.Input != null)
                    {
                        EVRInputError err = OpenVR.Input.UpdateActionState(rawActiveActionSetArray, activeActionSetSize);
                        if (err != EVRInputError.None)
                            Debug.LogError("<b>[SteamVR]</b> UpdateActionState error: " + err.ToString());
                        //else Debug.Log("Action sets activated: " + activeActionSets.Length);
                    }
                }
                else
                {
                    //Debug.LogWarning("No sets active");
                }
            }
        }

        public static void SetChanged()
        {
            changed = true;
        }

        private static int GetNewArraySize()
        {
            int size = 0;

            SteamVR_Input_Sources[] sources = SteamVR_Input_Source.GetAllSources();
            for (int actionSetIndex = 0; actionSetIndex < SteamVR_Input.actionSets.Length; actionSetIndex++)
            {
                SteamVR_ActionSet set = SteamVR_Input.actionSets[actionSetIndex];

                for (int sourceIndex = 0; sourceIndex < sources.Length; sourceIndex++)
                {
                    SteamVR_Input_Sources source = sources[sourceIndex];

                    if (set.ReadRawSetActive(source))
                    {
                        size++;
                    }
                }
            }

            return size;
        }

        private static void UpdateActionSetsArray()
        {
            int newArraySize = GetNewArraySize();
            if (poolActiveActionSetArrays.ContainsKey(newArraySize) == false)
            {
                poolActiveActionSetArrays[newArraySize] = new VRActiveActionSet_t[newArraySize];
            }

            int arrayIndex = 0;
            SteamVR_Input_Sources[] sources = SteamVR_Input_Source.GetAllSources();

            for (int actionSetIndex = 0; actionSetIndex < SteamVR_Input.actionSets.Length; actionSetIndex++)
            {
                SteamVR_ActionSet set = SteamVR_Input.actionSets[actionSetIndex];

                for (int sourceIndex = 0; sourceIndex < sources.Length; sourceIndex++)
                {
                    SteamVR_Input_Sources source = sources[sourceIndex];

                    if (set.ReadRawSetActive(source))
                    {
                        poolActiveActionSetArrays[newArraySize][arrayIndex].ulActionSet = set.handle;
                        poolActiveActionSetArrays[newArraySize][arrayIndex].nPriority = set.ReadRawSetPriority(source);
                        poolActiveActionSetArrays[newArraySize][arrayIndex].ulRestrictedToDevice = SteamVR_Input_Source.GetHandle(source);

                        arrayIndex++;
                    }
                }
            }

            changed = false;
            currentArraySize = newArraySize;

            if (Application.isEditor || updateDebugTextInBuilds)
                UpdateDebugText();
        }

        public static SteamVR_ActionSet GetSetFromHandle(ulong handle)
        {
            for (int actionSetIndex = 0; actionSetIndex < SteamVR_Input.actionSets.Length; actionSetIndex++)
            {
                SteamVR_ActionSet set = SteamVR_Input.actionSets[actionSetIndex];
                if (set.handle == handle)
                    return set;
            }

            return null;
        }

        public static string debugActiveSetListText;
        public static bool updateDebugTextInBuilds = false;
        private static void UpdateDebugText()
        {
            StringBuilder stringBuilder = new StringBuilder();

            for (int activeIndex = 0; activeIndex < rawActiveActionSetArray.Length; activeIndex++)
            {
                VRActiveActionSet_t set = rawActiveActionSetArray[activeIndex];
                stringBuilder.Append(set.nPriority);
                stringBuilder.Append("\t");
                stringBuilder.Append(SteamVR_Input_Source.GetSource(set.ulRestrictedToDevice));
                stringBuilder.Append("\t");
                stringBuilder.Append(GetSetFromHandle(set.ulActionSet).GetShortName());
                stringBuilder.Append("\n");
            }

            debugActiveSetListText = stringBuilder.ToString();
        }
    }
}