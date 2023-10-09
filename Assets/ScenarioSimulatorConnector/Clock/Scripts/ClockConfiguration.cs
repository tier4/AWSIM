using System.Linq;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Clock Configuration.
    /// Clock Configuration asset file should be present in project as ScriptableObject asset in 'Resources' folder.
    /// </summary>
    [CreateAssetMenu(fileName = "ClockConfiguration", menuName = "AWSIM/Configuration/Clock", order = 0)]
    public class ClockConfiguration : ScriptableObject
    {
        [TextArea(3, 10)][SerializeField] string description = "";

        #region [Settings]

        [Header("Settings")]
        public SourceType ClockSourceType = SourceType.UNITY;

        #endregion

        #region [Variables]
        
        [Header("Collection")]
        public SourceAsset[] Sources = default;

        #endregion

        #region [Public Methods]

        public ITimeSource GetTimeSource(SourceType type)
        {
            SourceAsset asset = Sources.FirstOrDefault(e => e.Type == type);
            if (asset.TimeSource != null && asset.TimeSource is ITimeSource)
            {
                return asset.TimeSource as ITimeSource;
            }

            return default;

        }

        #endregion

        [System.Serializable]
        public struct SourceAsset
        {
            public SourceType Type;
            public ScriptableObject TimeSource;
        }

        public enum SourceType
        {
            UNITY,
            SS2
        }
    }

}