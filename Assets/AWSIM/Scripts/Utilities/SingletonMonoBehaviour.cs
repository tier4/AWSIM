using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Apply a singleton to MonoBehaviour
    /// </summary>
    /// <typeparam name="T">Types to be Singleton</typeparam>
    public class SingletonMonoBehaviour<T> : MonoBehaviour where T : MonoBehaviour
    {
        static T instance;

        /// <summary>
        /// Returns the instance of this singleton
        /// </summary>
        public static T Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = (T)FindObjectOfType(typeof(T));

                    if (instance == null)
                    {
                        Debug.LogError("An instance of " + typeof(T) +
                            "is needed in the scene, but there is none.");
                    }
                }

                return instance;
            }
        }
    }
}