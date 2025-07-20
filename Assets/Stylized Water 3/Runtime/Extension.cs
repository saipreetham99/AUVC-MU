// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

namespace StylizedWater3
{
    public class Extension
    {
        private static Extension[] catalogue = new Extension[]
        {
            new Extension(ID.DynamicEffects, "Dynamic Effects", "Enables advanced effects to be projected onto the water surface. Such as boat wakes, ripples and shoreline waves.", 299321), 
            new Extension(ID.UnderwaterRendering, "Underwater Rendering", "Extends the shader with underwater rendering, by seamlessly blending the water with post processing effects.", 322081), 
            //new Extension(ID.Flowmaps, "Flowmaps", "Adds directional flow to water surfaces", 0),
            //new Extension(ID.Physics, "Physics", "Buoyancy physics to make Rigidbodies float in a natural way", 0),
        };
        
        protected Extension(){}

        protected Extension(ID id, string name, string description, int assetStoreID)
        {
            this.id = id;
            this.name = name;
            this.description = description;
            this.assetStoreID = assetStoreID;
        }
        
        public enum ID
        {
            Unknown,
            DynamicEffects,
            UnderwaterRendering,
            Flowmaps,
            Physics,
            Simulations,
            OceanWaves
        }

        public ID id = ID.Unknown;
        public string name;
        public string description;
        public int assetStoreID;
        public Texture2D icon;

        public string version;
        public string minBaseVersion;
        
        public static Extension[] installed;
        public static Extension[] available;
        
        public void CreateIcon(string data)
        {
            byte[] bytes = System.Convert.FromBase64String(data);

            icon = new Texture2D(32, 32, TextureFormat.RGBA32, false, false);
            icon.LoadImage(bytes, true);
        }
        
        #if UNITY_EDITOR
        //[UnityEditor.Callbacks.DidReloadScripts]
        [InitializeOnLoadMethod]
        private static void GetInstalled()
        {
            var allTypes = new List<System.Type>();
            var assemblies = System.AppDomain.CurrentDomain.GetAssemblies();
                
            foreach (var assembly in assemblies)
            {
                Type[] types = assembly.GetTypes();
                foreach (Type type in types)
                {
                    if (type.IsAbstract) continue;

                    if (type.IsSubclassOf(typeof(Extension)))
                        allTypes.Add(type);
                }
            }

            installed = new Extension[allTypes.Count];
            for (int i = 0; i < allTypes.Count; i++)
            {
                installed[i] = Activator.CreateInstance(allTypes[i]) as Extension;
                //installed[i] = Convert.ChangeType(typeof(Extension), allTypes[i]) as Extension;
                
                //Debug.Log($"Found installed extension: {allTypes[i]}");
            }

            for (int i = 0; i < installed.Length; i++)
            {
                installed[i].Load();
            }
            
            List<Extension> notInstalledList = new List<Extension>();

            for (int i = 0; i < catalogue.Length; i++)
            {
                //Get installed
                Extension extension = Get(catalogue[i].id);

                //Not installed
                if (extension == null)
                {
                    notInstalledList.Add(catalogue[i]);
                }
            }
            
            available = notInstalledList.ToArray();
            //Debug.Log($"{available.Length} extensions available");
        }
        #endif
        
        public virtual void Load(){}

        protected static Extension Get(ID id)
        {
            return installed.FirstOrDefault(e => e.id == id);
        }
    }
}