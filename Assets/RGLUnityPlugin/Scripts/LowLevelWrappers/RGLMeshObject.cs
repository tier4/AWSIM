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
    /// Contains information about RGL entity and has mesh identifier which refers to RGLMesh identifier.
    /// </summary>
    public class RGLObject
    {
        public string Identifier;
        public int MeshIdentifier;
        public Func<Matrix4x4> GetLocalToWorld;
        public GameObject RepresentedGO;

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
    /// RGL counterpart of Unity Mesh.
    /// Contains information about RGL mesh.
    /// Can be constructed in multiple ways - e.g. from collider or a (non-skinned) mesh.
    /// </summary>
    public class RGLMesh
    {
        public int Identifier;
        public Mesh Mesh;

        public IntPtr rglMesh = IntPtr.Zero;

        public int SubscribersCounter = 0;

        public override int GetHashCode()
        {
            return Identifier;
        }

        public override bool Equals(object obj)
        {
            return obj is RGLMesh rglMesh && Identifier.Equals(rglMesh.Identifier);
        }
    }

    /// <summary>
    /// Some objects (such as NPC) use skinned meshes, which needs to be constantly updated by the Unity side.
    /// </summary>
    public class RGLSkinnedMesh : RGLMesh
    {
        public SkinnedMeshRenderer SkinnedMeshRenderer;

        public void UpdateSkinnedMesh()
        {
            SkinnedMeshRenderer.BakeMesh(Mesh, true);
        }
    }
}