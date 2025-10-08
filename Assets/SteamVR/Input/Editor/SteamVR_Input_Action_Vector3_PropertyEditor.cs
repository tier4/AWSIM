﻿using UnityEditor;
using UnityEngine;

using System.CodeDom;
using Microsoft.CSharp;
using System.IO;
using System.CodeDom.Compiler;

using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using System.Linq.Expressions;
using System;


namespace Valve.VR
{
    [CustomPropertyDrawer(typeof(SteamVR_Action_Vector3))]
    public class SteamVR_Input_Action_Vector3_PropertyEditor : SteamVR_Input_Action_GenericPropertyEditor<SteamVR_Action_Vector3>
    {
    }
}