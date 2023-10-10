using UnityEngine;

namespace AWSIM
{
public class ObjectClassification : MonoBehaviour
{
    // Attach this script to target object to enable perception result sensor
    public enum ObjectType
    {
        UNKNOWN,
        CAR,
        TRUCK,
        BUS,
        TRAILER,
        MOTORCYCLE,
        BICYCLE,
        PEDESTRIAN
    }

    public ObjectType objectType;
}
}