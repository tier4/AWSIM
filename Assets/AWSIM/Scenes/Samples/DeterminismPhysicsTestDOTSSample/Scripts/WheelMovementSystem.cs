using Unity.Entities;
using Unity.Transforms;
using Unity.Jobs;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Mathematics;

namespace AWSIM.PhysicsTest
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    partial class WheelMovementSystem : SystemBase
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

            if (_currTime >= 4f && _currTime < 14f)
            {
                speed = 1.5f;
            }
            else if (_currTime >= 14f && _currTime < 24f)
            {
                speed = 0f;
            }
            else if (_currTime >= 24f && _currTime < 34f)
            {
                speed = -1.5f;
            }
            else
            {
                speed = 0f;
            }

            _currTime += Time.DeltaTime;

            Entities
                .WithAll<WheelIdData>()
                .ForEach((ref PhysicsVelocity physicsVelocity, ref PhysicsMass physicsMass, in Rotation rotation) =>
                {
                    float3 direction = math.normalize(math.mul(rotation.Value, new float3(1f, 0f, 0f)));
                    float3 forwardDirection = math.normalize(math.mul(quaternion.AxisAngle(math.up(), math.radians(-90)), direction));

                    float3 impulse = forwardDirection * deltaTime * speed;

                    physicsVelocity.ApplyLinearImpulse(physicsMass, impulse);

                }).Run();
        }
    }
}