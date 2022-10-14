using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM.Samples
{
    public class TrafficLightSample : MonoBehaviour
    {
        [SerializeField] TrafficLight trafficLight;

        // Start is called before the first frame update
        void Start()
        {
            StartCoroutine(Test());
        }

        IEnumerator Test()
        {
            TrafficLight.BulbData input;
            TrafficLight.BulbData[] inputs;
            Debug.Log("--- Start Traffic Light Test ---");
            Debug.Log("Green");
            // create input bulb data.
            input = new TrafficLight.BulbData(TrafficLight.BulbType.GREEN_BULB, TrafficLight.BulbColor.GREEN, TrafficLight.BulbStatus.SOLID_ON);
            
            // applay input blub data.
            trafficLight.SetBulbData(input);
            yield return new WaitForSeconds(4f);

            Debug.Log("Yellow");
            inputs = new TrafficLight.BulbData[]
            {
                new TrafficLight.BulbData(TrafficLight.BulbType.GREEN_BULB, TrafficLight.BulbColor.GREEN, TrafficLight.BulbStatus.SOLID_OFF),
                new TrafficLight.BulbData(TrafficLight.BulbType.YELLOW_BULB, TrafficLight.BulbColor.YELLOW, TrafficLight.BulbStatus.SOLID_ON)
            };
            trafficLight.SetBulbData(inputs);
            yield return new WaitForSeconds(4f);

            Debug.Log("Yello flashing");
            input = new TrafficLight.BulbData(TrafficLight.BulbType.YELLOW_BULB, TrafficLight.BulbColor.YELLOW, TrafficLight.BulbStatus.FLASHING);
            trafficLight.SetBulbData(input);
            yield return new WaitForSeconds(4f);

            Debug.Log("Red");
            inputs = new TrafficLight.BulbData[]
            {
                new TrafficLight.BulbData(TrafficLight.BulbType.YELLOW_BULB, TrafficLight.BulbColor.YELLOW, TrafficLight.BulbStatus.SOLID_OFF),
                new TrafficLight.BulbData(TrafficLight.BulbType.RED_BULB, TrafficLight.BulbColor.RED, TrafficLight.BulbStatus.SOLID_ON)
            };
            trafficLight.SetBulbData(inputs);
            yield return new WaitForSeconds(4f);

            Debug.Log("Arrow");
            inputs = new TrafficLight.BulbData[]
            {
                new TrafficLight.BulbData(TrafficLight.BulbType.UP_ARROW_BULB, TrafficLight.BulbColor.WHITE, TrafficLight.BulbStatus.SOLID_ON),
                new TrafficLight.BulbData(TrafficLight.BulbType.RIGHT_ARROW_BULB, TrafficLight.BulbColor.WHITE, TrafficLight.BulbStatus.SOLID_ON)
            };
            trafficLight.SetBulbData(inputs);
            Debug.Log("--- End Traffic Light Test ---");
        }
    }
}