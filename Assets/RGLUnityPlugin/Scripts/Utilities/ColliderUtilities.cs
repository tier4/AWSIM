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
using System.Collections.Generic;
using UnityEngine;

namespace RGLUnityPlugin
{
    /// <summary>
    /// Utilities to help with translating collider components into RGL concepts - meshes & transforms.
    /// </summary>
    public class ColliderUtilities
    {
        /// <summary>
        /// Translates Collider Component into a Mesh
        /// Mesh Collider => Its own mesh
        /// Primitive Collider => Generates primitive mesh
        /// </summary>
        public static Mesh GetMeshForCollider(Collider collider)
        {
            var type = collider.GetType();
            return type == typeof(MeshCollider)
                ? ((MeshCollider) collider).sharedMesh
                : GetPrimitiveMesh(ColliderPrimitiveType[type]);
        }

        /// <summary>
        /// Provides a matrix that describes collider component specific settings (center, box size, capsule radius, etc.)
        /// as a localToWorld (collider component to game object) Matrix.
        /// </summary>
        public static Matrix4x4 GetColliderTransformMatrix(Collider collider)
        {
            var type = collider.GetType();
            if (type == typeof(MeshCollider))
            {
                return Matrix4x4.identity;
            }

            if (type == typeof(BoxCollider))
            {
                var bc = (BoxCollider) collider;
                return Matrix4x4.TRS(
                    bc.center,
                    Quaternion.identity,
                    bc.size);
            }

            if (type == typeof(CapsuleCollider))
            {
                var cc = (CapsuleCollider) collider;
                return Matrix4x4.TRS(
                    cc.center,
                    RotateToYAxis[cc.direction],
                    new Vector3(cc.radius / 0.5f, cc.height / 2.0f, cc.radius / 0.5f)
                );
            }

            if (type == typeof(SphereCollider))
            {
                var cs = (SphereCollider) collider;
                var s = cs.radius / 0.5f;
                return Matrix4x4.TRS(
                    cs.center,
                    Quaternion.identity,
                    new Vector3(s, s, s));
            }

            Debug.LogError($"Unsupported primitive type: {type.Name}");
            return Matrix4x4.identity;
        }

        private static Mesh GetPrimitiveMesh(PrimitiveType type)
        {
            if (!primitiveMeshes.ContainsKey(type))
            {
                var gameObject = GameObject.CreatePrimitive(type);
                var mesh = gameObject.GetComponent<MeshFilter>().sharedMesh;
                GameObject.Destroy(gameObject);
                primitiveMeshes[type] = mesh;
            }

            return primitiveMeshes[type];
        }

        /// Capsule colliders in the wild can be oriented along any axis.
        /// Capsule mesh, on the other hand, is always oriented along Y axis.
        /// This dictionary maps (capsule collider direction) => (rotation to orient it identically as capsule mesh).
        private static readonly Dictionary<int, Quaternion> RotateToYAxis = new Dictionary<int, Quaternion>
        {
            {0, Quaternion.AngleAxis(-90, Vector3.forward)}, // X-oriented capsule
            {1, Quaternion.identity},
            {2, Quaternion.AngleAxis(+90, Vector3.right)} // Z-oriented capsule
        };

        private static readonly Dictionary<Type, PrimitiveType> ColliderPrimitiveType =
            new Dictionary<Type, PrimitiveType>
            {
                {typeof(BoxCollider), PrimitiveType.Cube},
                {typeof(CapsuleCollider), PrimitiveType.Cylinder},
                {typeof(SphereCollider), PrimitiveType.Sphere},
            };

        private static Dictionary<PrimitiveType, Mesh> primitiveMeshes = new Dictionary<PrimitiveType, Mesh>();
    }
}