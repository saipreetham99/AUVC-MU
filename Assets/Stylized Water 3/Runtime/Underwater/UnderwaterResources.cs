// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using UnityEngine;

namespace StylizedWater3.UnderwaterRendering
{
    //[CreateAssetMenu(fileName = "UnderwaterResources", menuName = "UnderwaterResources", order = 0)]
    /// <summary>
    /// Shared resources, ensures they're included in a build when the render feature is in use
    /// </summary>
    public class UnderwaterResources : ScriptableObject
    {
        [Header("Shaders")]
        public Shader underwaterShader;
        public Shader waterlineShader;
        public Shader watermaskShader;
        public Shader postProcessShader;
        public Shader distortionShader;

        [Header("Textures")]
        public Texture2D distortionNoise;

        [Header("Meshes")]
        [Tooltip("Used for the world-space distortion mode")]
        public Mesh geoSphere;

        private const string mainGUID = "185c96ef53e9cf14bbc913f624752e24";
        
        public static UnderwaterResources Find()
        {
            #if UNITY_EDITOR
            string path = UnityEditor.AssetDatabase.GUIDToAssetPath(mainGUID);

            if (path == string.Empty)
            {
                Debug.LogError("The UnderwaterResources asset could not be found with the GUID " + mainGUID + ". Was it not imported? Meta file deleted?");
                return null;
            }
            
            UnderwaterResources r = UnityEditor.AssetDatabase.LoadAssetAtPath(path, typeof(UnderwaterResources)) as UnderwaterResources;

            return r;
            #else
            return null;
            #endif
        }
    }
}