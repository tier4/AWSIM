// Copyright 2022 Robotec.ai.
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

namespace RGLUnityPlugin
{
    /// <summary>
    /// RGL counterpart of Unity GameObjects.
    /// Contains all information needed by RGL native module.
    /// Can be constructed in multiple ways - e.g. from collider or a (non-skinned) mesh.
    /// </summary>
    public class RGLObject
    {
        public string Identifier;
        public Mesh Mesh;
        public Func<Matrix4x4> GetLocalToWorld;
        public GameObject RepresentedGO;

        // TODO(prybicki): I'm a bit dubious about these fields here, seems to violate SRP
        public IntPtr rglMesh;
        public IntPtr rglEntity;

        public override int GetHashCode()
        {
            return Identifier.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return obj is RGLObject rglObject && Identifier.Equals(rglObject.Identifier);
        }
    }

    /// <summary>
    /// Some objects (such as NPC) use skinned meshes, which needs to be constantly updated by the Unity side.
    /// </summary>
    public class RGLSkinnedObject : RGLObject
    {
        public SkinnedMeshRenderer SkinnedMeshRenderer;

        public void UpdateSkinnedMesh()
        {
            SkinnedMeshRenderer.BakeMesh(Mesh, true);
        }
    }
}