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

using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Awsim.Common;

namespace Awsim.UI
{
    public class AwsimCanvas : MonoBehaviour
    {
        const string _scrollViewShownPrefsKey = "scrollViewShownPrefsKey";
        const string _uiScalePrefsKey = "uiScalePrefsKey";
        const bool _defaultScrollViewShown = true;
        const float _defaultUIScale = 1.0f;

        [SerializeField] CanvasScaler _canvasScaler;
        [SerializeField] RectTransform _scrollViewRectTransform;
        [SerializeField] GameObject _viewPortObject;
        [SerializeField] GameObject _openScrollViewButtonObject;
        [SerializeField] GameObject _closeScrollViewButtonObject;
        [SerializeField] Slider _uiScaleSlider;
        [SerializeField] TMP_Text _uiScaleText;
        [SerializeField] Transform _toggleButtonParent;
        [SerializeField] GameObject _toggleButtonPrefab;
        [SerializeField] Button _resetButton;
        [SerializeField] Button _closeScrollViewButton;
        [SerializeField] Button _openScrollViewButton;
        [SerializeField] UIWindow[] _uiWindows;
        ToggleButton[] _toggleButtons;
        Vector2 _defaultReferenceResolution;

        public void Initialize()
        {
            // UI scaling.
            _defaultReferenceResolution = _canvasScaler.referenceResolution;
            _uiScaleSlider.onValueChanged.AddListener(x => SetUIScale(x));
            var scale = LoadUIScale();
            SetUIScale(scale);

            // Reset button.
            _resetButton.onClick.AddListener(() =>
            {
                AllReset();
            });

            // Open and close scroll view button.
            _closeScrollViewButton.onClick.AddListener(() =>
            {
                CloseScrollView();
            });

            _openScrollViewButton.onClick.AddListener(() =>
            {
                OpenScrollView();
            });

            // 

            // Toggle scroll view.
            var isScrollViewShown = LoadScrollViewShown();
            SwitchScrollView(isScrollViewShown);
            _toggleButtons = new ToggleButton[_uiWindows.Length];

            for (int i = 0; i < _uiWindows.Length; i++)
            {
                var initialToggleOn = _uiWindows[i].LoadShown();

                // Instantiate toggle button.
                var instance = Instantiate(_toggleButtonPrefab);
                instance.transform.SetParent(_toggleButtonParent.transform);
                instance.transform.localScale = Vector3.one;
                var toggleButton = instance.GetComponent<ToggleButton>();
                _toggleButtons[i] = toggleButton;

                // Initialize ui window.
                _uiWindows[i].Initialize(() => toggleButton.SetIsOn(false));

                // Initialize toggle button.
                toggleButton.Initialize(_uiWindows[i].WindowName, initialToggleOn, _uiWindows[i].Open, _uiWindows[i].Close);
            }

            foreach (var uiWindow in _uiWindows)
            {
                uiWindow.OnStart();
            }
        }

        public void OnUpdate()
        {
            foreach (var uiWindow in _uiWindows)
            {
                uiWindow.OnUpdate();
            }
        }

        public void OpenScrollView()
        {
            SwitchScrollView(true);
        }

        public void CloseScrollView()
        {
            SwitchScrollView(false);
        }

        public void AllReset()
        {
            for (int i = 0; i < _uiWindows.Length; i++)
            {
                _uiWindows[i].ResetUI();
                _toggleButtons[i].SetIsOn(_uiWindows[i].DefaultToggleIsOn);
            }

            SetUIScale(_defaultUIScale);
        }

        public void SetUIScale(float scale)
        {
            _uiScaleSlider.value = scale;
            _canvasScaler.referenceResolution = _defaultReferenceResolution * scale;
            var displayValue = 1 + (1 - scale);
            _uiScaleText.text = "x " + displayValue.ToString("F2");
            SaveUIScale(scale);
        }

        void SwitchScrollView(bool isOpen)
        {
            _viewPortObject.gameObject.SetActive(isOpen);
            _openScrollViewButtonObject.gameObject.SetActive(!isOpen);
            _closeScrollViewButtonObject.gameObject.SetActive(isOpen);

            SaveScrollViewShown(isOpen);

            if (isOpen)
                _scrollViewRectTransform.anchoredPosition = new Vector2(0, _scrollViewRectTransform.anchoredPosition.y);
            else
                _scrollViewRectTransform.anchoredPosition = new Vector2(_scrollViewRectTransform.sizeDelta.x, _scrollViewRectTransform.anchoredPosition.y);
        }

        bool LoadScrollViewShown()
        {
            return PlayerPrefsX.GetBool(_scrollViewShownPrefsKey, _defaultScrollViewShown);
        }

        void SaveScrollViewShown(bool shown)
        {
            PlayerPrefsX.SetBool(_scrollViewShownPrefsKey, shown);
        }

        float LoadUIScale()
        {
            return PlayerPrefs.GetFloat(_uiScalePrefsKey, _defaultUIScale);
        }

        void SaveUIScale(float scale)
        {
            PlayerPrefs.SetFloat(_uiScalePrefsKey, scale);
        }
    }
}