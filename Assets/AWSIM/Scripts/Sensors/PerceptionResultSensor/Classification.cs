using UnityEngine;

namespace AWSIM
{
public class Classification : MonoBehaviour
{
    // use this for object sensor
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