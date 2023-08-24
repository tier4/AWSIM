using System.Collections.Generic;
using UnityEngine;

namespace RGLUnityPlugin
{
    public class RGLTextureSharingManager
    {
        private static Dictionary<int, RGLTexture> sharedTextures = new Dictionary<int, RGLTexture>(); // <Identifier, RGLTexture>
        private static Dictionary<int, int> sharedTexturesUsageCount = new Dictionary<int, int>(); // <RGLTexture Identifier, count>
        
        public static RGLTexture RegisterRGLTextureInstance(Texture2D texture)
        {
            var textureID = texture.GetInstanceID();
                
            if(!sharedTextures.ContainsKey(textureID))
            {
                var rglTextureToAdd = new RGLTexture(texture, textureID);
                sharedTextures.Add(textureID, rglTextureToAdd);
                sharedTexturesUsageCount.Add(textureID, 0);                    
            }                    
            
            sharedTexturesUsageCount[textureID] += 1;

            return sharedTextures[textureID];
        }

        public static void UnregisterRGLTextureInstance(RGLTexture rglTexture)
        {
            var textureId = rglTexture.Identifier;
            if (sharedTextures[textureId] is null)
            {
                Debug.LogWarning($"Trying to unregister absent in RGLTextureSharingManager texture of id: {textureId}, ignoring request");
                return;
            }

            sharedTexturesUsageCount[textureId]--;
            if (sharedTexturesUsageCount[textureId] == 0)
            {
                sharedTextures[textureId].DestroyInRGL();
                sharedTextures.Remove(textureId);
                sharedTextures.Remove(textureId);
            }
        }

        public static void Clear()
        {
            foreach (var mesh in sharedTextures)
            {
                mesh.Value.DestroyInRGL();
            }
        }
    }
}
