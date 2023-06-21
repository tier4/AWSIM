using UnityEngine;
using Unity.Entities;
using Unity.Transforms;
using Unity.Jobs;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Mathematics;

namespace AWSIM.PhysicsTest
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    partial class BallMovementSystem : SystemBase
    {
        private float _currTime = 0f;

        protected override void OnStartRunning()
        {
            _currTime = 0f;
        }

        protected override void OnUpdate()
        {
            float deltaTime = Time.DeltaTime;
            float speed = 0f;

            if (_currTime >= 2f && _currTime < 7f)
            {
                speed = 1.5f;
            }
            else if (_currTime >= 7f && _currTime < 12f)
            {
                speed = 0f;
            }
            else if (_currTime >= 12f && _currTime < 17f)
            {
                speed = -1.5f;
            }
            else
            {
                speed = 0f;
            }

            _currTime += Time.DeltaTime;

            float3 direction = new float3(0f, 0f, 1f);

            Entities
                .WithAll<BallTag>()
                .ForEach((ref PhysicsVelocity physicsVelocity, ref PhysicsMass physicsMass) =>
                {
                    float3 impulse = direction * deltaTime * speed;

                    physicsVelocity.ApplyLinearImpulse(physicsMass, impulse);
                }).Run();
        }
    }
}
