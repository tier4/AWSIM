using UnityEngine;

public class CarLightsController : MonoBehaviour
{
    public Light leftHeadlight;
    public Light rightHeadlight;

    void Start()
    {
        // Ensure headlights are always on when the game starts
        leftHeadlight.enabled = true;
        rightHeadlight.enabled = true;
    }
}
