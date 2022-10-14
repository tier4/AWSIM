using AWSIM.Lanelet;
using UnityEngine;

namespace AWSIM.PointCloudMapping
{
    [UnityEditor.AssetImporters.ScriptedImporter(1, "osm")]
    public class OsmImporter : UnityEditor.AssetImporters.ScriptedImporter
    {
        public override void OnImportAsset(UnityEditor.AssetImporters.AssetImportContext ctx)
        {
            var container = ScriptableObject.CreateInstance<OsmDataContainer>();
            container.Data = OsmData.Read(ctx.assetPath);
            ctx.AddObjectToAsset("OSM Data", container);
            ctx.SetMainObject(container);
        }
    }
}
