// using System.Collections;
// using UnityEngine;
// using UnityEngine.UI;

// public class ChangeImage : MonoBehaviour
// {
//     public Image displayImage;  // Reference to the Image component
//     public Sprite newSprite;   // The sprite you want to display after 10 seconds
//     public Sprite lastSprite; //
//     public float delay = 10f;  // Time delay before changing the image
//     public float delay2 = 3f;  // Time delay before changing the image

//     private void Start()
//     {
//         // Start the coroutine to change the image after a delay
//         StartCoroutine(ChangeImageAfterDelay());
//     }

//     private IEnumerator ChangeImageAfterDelay()
//     {
//         // Wait for the specified delay
//         yield return new WaitForSeconds(delay);

//         // Change the sprite of the Image component
//         if (displayImage != null && newSprite != null)
//         {
//             displayImage.sprite = newSprite;
//         }
//     }
// }
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class ChangeImage : MonoBehaviour
{
    public Image displayImage;  // Reference to the Image component
    public Sprite newSprite;   // The sprite you want to display after delay
    public Sprite lastSprite;  // The final sprite after delay2
    public float delay = 10f;  // Time delay before changing to newSprite
    public float delay2 = 3f;  // Time delay before changing to lastSprite

    private void Start()
    {
        // Start the coroutine to change the image after a delay
        StartCoroutine(ChangeImageSequence());
    }

    private IEnumerator ChangeImageSequence()
    {
        // Wait for the first delay
        yield return new WaitForSeconds(delay);

        // Change to the new sprite
        if (displayImage != null && newSprite != null)
        {
            displayImage.sprite = newSprite;
        }

        // Wait for the second delay
        yield return new WaitForSeconds(delay2);

        // Change to the last sprite
        if (displayImage != null && lastSprite != null)
        {
            displayImage.sprite = lastSprite;
        }
    }
}
