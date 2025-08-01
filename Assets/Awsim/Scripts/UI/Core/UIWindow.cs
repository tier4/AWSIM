// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using Awsim.Common;

namespace Awsim.UI
{
    /// <summary>
    /// UI's window class. Please create UI by inheriting from this class.
    /// Window can be dragged and closed.
    /// </summary>
    public class UIWindow : MonoBehaviour, IDragHandler, IBeginDragHandler, IEndDragHandler, IPointerEnterHandler, IPointerExitHandler
    {
        /// <summary>
        /// For player prefs local save data.
        /// </summary>
        public string WindowName => _windowName;

        /// <summary>
        /// UiWindow's rect transform.
        /// </summary>
        public RectTransform RectTransform => _rectTransform;

        /// <summary>
        /// UIWindow close button.
        /// </summary>
        public Button CloseButton => _closeButton;

        /// <summary>
        /// Default toggle value. (show or close)
        /// </summary>
        public bool DefaultToggleIsOn => _defaultToggleIsOn;

        /// <summary>
        /// Default ui window position.
        /// </summary>
        public Vector2 DefaultAnchoredPosition => _defaultAnchoredPosition;

        /// <summary>
        /// Is this window's contents shown currently?
        /// </summary>
        protected bool Shown { get; private set; } = false;

        public const string PosxPrefixPrefsKey = "posxPrefixPrefsKey";
        public const string PosyPrefixPrefsKey = "posyPrefixPrefsKey";
        public const string ShownPrefixPrefsKey = "shownPrefixPrefsKey";

        [Header("UIWindow settings")]
        [SerializeField] string _windowName = string.Empty;
        [SerializeField] protected bool _defaultToggleIsOn = false;
        [SerializeField] protected Vector2 _defaultAnchoredPosition = Vector2.zero;
        [SerializeField] GameObject _contents;
        [SerializeField] Button _closeButton;
        RectTransform _rectTransform;
        Vector2 _mouseOffset;

        /// <summary>
        /// Initialize UIWindow.
        /// </summary>
        /// <param name="anchoredInitialPosition"></param>
        public void Initialize(Action onCloseButtonClickAction)
        {
            _rectTransform = GetComponent<RectTransform>();
            _closeButton.gameObject.SetActive(false);

            var loadedShown = LoadShown();
            var loadedPos = LoadPosition();

            SetPosition(loadedPos);

            if (loadedShown)
                Open();
            else
                Close();

            _closeButton.onClick.AddListener(() =>
            {
                onCloseButtonClickAction();
                Close();
            });
        }

        public void ResetUI()
        {
            SetPosition(DefaultAnchoredPosition);
        }

        /// <summary>
        /// Set UIWindow position.
        /// </summary>
        /// <param name="anchoredPosition"></param>
        public void SetPosition(Vector2 anchoredPosition)
        {
            // Set initial position.
            _rectTransform.anchoredPosition = anchoredPosition;

            SavePosition(anchoredPosition);
        }

        /// <summary>
        /// Open UIWindow's contents.
        /// </summary>
        public void Open()
        {
            Shown = true;
            _contents.SetActive(true);
            SaveShown(true);
        }

        /// <summary>
        /// Close UIWindow's contents.
        /// </summary>
        public void Close()
        {
            Shown = false;
            _contents.SetActive(false);
            SaveShown(false);
        }

        public void OnDrag(PointerEventData eventData)
        {
            if (!Shown)
                return;

            transform.position = (Vector2)eventData.position + _mouseOffset;
        }

        public void OnBeginDrag(PointerEventData eventData)
        {
            if (!Shown)
                return;

            _mouseOffset = transform.position - Input.mousePosition;
        }
        public void OnEndDrag(PointerEventData eventData)
        {
            if (!Shown)
                return;

            SavePosition(_rectTransform.anchoredPosition);
        }

        public void OnPointerEnter(PointerEventData eventData)
        {
            if (!Shown)
                return;

            _closeButton.gameObject.SetActive(true);
        }

        public void OnPointerExit(PointerEventData eventData)
        {
            if (!Shown)
                return;

            _closeButton.gameObject.SetActive(false);
        }

        public virtual void OnStart()
        {

        }

        public virtual void OnUpdate()
        {

        }

        public bool LoadShown()
        {
            return PlayerPrefsX.GetBool(ShownPrefixPrefsKey + WindowName, _defaultToggleIsOn);
        }


        void SavePosition(Vector2 anchoredPosition)
        {
            PlayerPrefs.SetFloat(PosxPrefixPrefsKey + WindowName, anchoredPosition.x);
            PlayerPrefs.SetFloat(PosyPrefixPrefsKey + WindowName, anchoredPosition.y);
        }

        void SaveShown(bool shown)
        {
            PlayerPrefsX.SetBool(ShownPrefixPrefsKey + WindowName, shown);
        }

        Vector2 LoadPosition()
        {
            var loadedPosX = PlayerPrefs.GetFloat(PosxPrefixPrefsKey + WindowName, _defaultAnchoredPosition.x);
            var loadedPosY = PlayerPrefs.GetFloat(PosyPrefixPrefsKey + WindowName, _defaultAnchoredPosition.y);

            return new Vector2(loadedPosX, loadedPosY);
        }
    }
}