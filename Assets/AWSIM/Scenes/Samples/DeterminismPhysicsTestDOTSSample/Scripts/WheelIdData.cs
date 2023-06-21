using Unity.Entities;

namespace AWSIM.PhysicsTest
{
    [GenerateAuthoringComponent]
    public struct WheelIdData : IComponentData
    {
        public int Id;
    }
}
