using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// UI class for RightScrollview.
    /// Open and Close functions are available.
    /// </summary>
    public class RightScrollViewUI : MonoBehaviour
    {
        [SerializeField] RectTransform rightScrollViewRect;
        [SerializeField] GameObject OpenUIButtonObj;
        [SerializeField] GameObject CloseUIBUttonObj;
        Vector3 initialPos;
        Vector3 closedPos = new Vector3(165, 0, 0); // take off the screen position

        /// <summary>
        /// open/close can be selected at initialization
        /// </summary>
        public bool opened = true;

        void Start()
        {
            initialPos = rightScrollViewRect.anchoredPosition;

            if (opened)
                OpenUI();
            else
                CloseUI();
        }

        /// <summary>
        /// Callback from OpenUIButton
        /// </summary>
        public void OpenUI()
        {
            opened = true;
            rightScrollViewRect.anchoredPosition = initialPos;
            OpenUIButtonObj.SetActive(false);
            CloseUIBUttonObj.SetActive(true);
        }

        /// <summary>
        /// Callback from CloseUIButton
        /// </summary>
        public void CloseUI()
        {
            opened = false;
            rightScrollViewRect.anchoredPosition = closedPos;
            OpenUIButtonObj.SetActive(true);
            CloseUIBUttonObj.SetActive(false);
        }
    }
}