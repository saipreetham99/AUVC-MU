// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

#if !UNITY_6000_0_OR_NEWER
#error [Stylized Water 3] Imported in a version older than Unity 6, all present script and shader compile errors are valid and not something to simply be fixed. Upgrade the project to Unity 6 to resolve them.
#endif

using System;
using System.Collections.Generic;
using System.Net;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEditor.PackageManager.Requests;
using PackageInfo = UnityEditor.PackageManager.PackageInfo;
using UnityEngine;
using Object = UnityEngine.Object;

namespace StylizedWater3
{
    public class AssetInfo
    {
        private const string THIS_FILE_GUID = "b15801d8ff7e9574288149dd4cebaa68";
        
        public const string ASSET_NAME = "Stylized Water 3";
        public const string ASSET_ID = "287769";
        public const string ASSET_ABRV = "SW3";

        public const string INSTALLED_VERSION = "3.2.1";
        
        private static readonly Dictionary<string, int> REQUIRED_PATCH_VERSIONS = new Dictionary<string, int>()
        {
            { "6000.0", 22 },
            { "6000.1", 3 }
        };
        public const string MIN_URP_VERSION = "17.0.3";

        public const string DOC_URL = "https://staggart.xyz/unity/stylized-water-3/sw3-docs/";
        public const string FORUM_URL = "https://discussions.unity.com/t/1542753";
        public const string EMAIL_URL = "mailto:contact@staggart.xyz?subject=Stylized Water 3";
        public const string DISCORD_INVITE_URL = "https://discord.gg/GNjEaJc8gw";

#if !URP //Enabled when com.unity.render-pipelines.universal is below MIN_URP_VERSION
        [InitializeOnLoad]
        sealed class PackageInstaller : Editor
        {
            [InitializeOnLoadMethod]
            public static void Initialize()
            {
                GetLatestCompatibleURPVersion();

                if (EditorUtility.DisplayDialog(ASSET_NAME + " v" + INSTALLED_VERSION, "This package requires the Universal Render Pipeline " + MIN_URP_VERSION + " or newer, would you like to install or update it now?", "OK", "Later"))
                {
					Debug.Log("Universal Render Pipeline <b>v" + lastestURPVersion + "</b> will start installing in a moment. Please refer to the URP documentation for set up instructions");
					Debug.Log("After installing and setting up URP, you must Re-import the Shaders folder!");
					
                    InstallURP();
                }
            }

            private static PackageInfo urpPackage;

            private static string lastestURPVersion;

#if SWS_DEV
            [MenuItem("SWS/Get latest URP version")]
#endif
            private static void GetLatestCompatibleURPVersion()
            {
                if(urpPackage == null) urpPackage = GetURPPackage();
                if(urpPackage == null) return;
                
                lastestURPVersion = urpPackage.versions.latestCompatible;
                
#if SWS_DEV
                Debug.Log("Latest compatible URP version: " + lastestURPVersion);
#endif
            }

            private static void InstallURP()
            {
                if(urpPackage == null) urpPackage = GetURPPackage();
                if(urpPackage == null) return;
                
                lastestURPVersion = urpPackage.versions.latestCompatible;

                AddRequest addRequest = Client.Add(URP_PACKAGE_ID + "@" + lastestURPVersion);
                
                //Update Core and Shader Graph packages as well, doesn't always happen automatically
                for (int i = 0; i < urpPackage.dependencies.Length; i++)
                {
#if SWS_DEV
                    Debug.Log("Updating URP dependency <i>" + urpPackage.dependencies[i].name + "</i> to " + urpPackage.dependencies[i].version);
#endif
                    addRequest = Client.Add(urpPackage.dependencies[i].name + "@" + urpPackage.dependencies[i].version);
                }
                
                //Wait until finished
                while(!addRequest.IsCompleted || addRequest.Status == StatusCode.InProgress) { }
                
                WaterShaderImporter.ReimportAll();
            }
        }
#endif
        
        public const string URP_PACKAGE_ID = "com.unity.render-pipelines.universal";

        public static PackageInfo GetURPPackage()
        {
            SearchRequest request = Client.Search(URP_PACKAGE_ID);
                
            while (request.Status == StatusCode.InProgress) { /* Waiting... */ }

            if (request.Status == StatusCode.Failure)
            {
                Debug.LogError("Failed to retrieve URP package from Package Manager...");
                return null;
            }

            return request.Result[0];
        }

        //Sorry, as much as I hate to intrude on an entire project, this is the only way in Unity to track importing or updating an asset
        public class ImportOrUpdateAsset : AssetPostprocessor
        {
            private static void OnPostprocessAllAssets(string[] importedAssets, string[] deletedAssets, string[] movedAssets, string[] movedFromPath)
            {
                for (int i = 0; i < importedAssets.Length; i++)
                {
                    OnPreProcessAsset(importedAssets[i]);
                }
                
                for (int i = 0; i < deletedAssets.Length; i++)
                {
                    OnPreProcessAsset(deletedAssets[i]);
                }
            }
            
            private static void OnPreProcessAsset(string m_assetPath)
            {
                //Importing/updating the Stylized Water 3 asset
                if (m_assetPath.EndsWith("Stylized Water 3/Editor/AssetInfo.cs") || m_assetPath.EndsWith("com.staggartcreations.stylizedwater3/Editor/AssetInfo.cs"))
                {
                    Installer.Initialize();
                    Installer.OpenWindowIfNeeded();
                }
                
                //These files change every version, so will trigger when updating or importing the first time
                if (m_assetPath.EndsWith("Extension.UnderwaterRendering.cs"))
                {
                    OnImportExtension("Underwater Rendering");
                }

                if (m_assetPath.EndsWith("Extension.DynamicEffects.cs"))
                {
                    OnImportExtension("Dynamic Effects");
                }
            }

            private static void OnImportExtension(string name)
            {
                Debug.Log($"[Stylized Water 3] {name} extension installed/deleted or updated. Reimporting water shader(s) to toggle integration.");
                
                //Re-import any .watershader3 files, since these depend on the installation state of extensions
                WaterShaderImporter.ReimportAll();
            }
        }
        
        public static bool MeetsMinimumVersion(string versionMinimum)
        {
            Version curVersion = new Version(INSTALLED_VERSION);
            Version minVersion = new Version(versionMinimum);

            return curVersion >= minVersion;
        }

        public static void OpenAssetStore(string url = null)
        {
            if (url == string.Empty) url = "https://assetstore.unity.com/packages/slug/" + ASSET_ID;
            
            Application.OpenURL(url + "?aid=1011l7Uk8&pubref=sw3editor");
        }

        public static void OpenReviewsPage()
        {
            Application.OpenURL($"https://assetstore.unity.com/packages/slug/{ASSET_ID}?aid=1011l7Uk8&pubref=sw3editor#reviews");
        }
        
        public static void OpenInPackageManager()
        {
            Application.OpenURL("com.unity3d.kharma:content/" + ASSET_ID);
        }
        
        public static void OpenForumPage()
        {
            Application.OpenURL(FORUM_URL + "/page-999");
        }
        
        public static string GetRootFolder()
        {
            //Get script path
            string scriptFilePath = AssetDatabase.GUIDToAssetPath(THIS_FILE_GUID);

            //Truncate to get relative path
            string rootFolder = scriptFilePath.Replace("Editor/AssetInfo.cs", string.Empty);

#if SWS_DEV
            //Debug.Log("<b>Package root</b> " + rootFolder);
#endif

            return rootFolder;
        }

        public static class VersionChecking
        {
            public static string MinRequiredUnityVersion = "undefined";

            public enum UnityVersionType
            {
                Release,
                Beta,
                Alpha
            }
            public static UnityVersionType unityVersionType;
            
            [InitializeOnLoadMethod]
            static void Initialize()
            {
                if (CHECK_PERFORMED == false)
                {
                    CheckForUpdate(false);
                    CHECK_PERFORMED = true;
                }
            }
            
            private static bool CHECK_PERFORMED
            {
                get => SessionState.GetBool("SW3_VERSION_CHECK_PERFORMED", false);
                set => SessionState.SetBool("SW3_VERSION_CHECK_PERFORMED", value);
            }
            
            public static string LATEST_VERSION
            {
                get => SessionState.GetString("SW3_LATEST_VERSION", INSTALLED_VERSION);
                set => SessionState.SetString("SW3_LATEST_VERSION", value);
            }
            public static bool UPDATE_AVAILABLE => new Version(LATEST_VERSION) > new Version(INSTALLED_VERSION);
            
            public static string GetUnityVersion()
            {
                string version = UnityEditorInternal.InternalEditorUtility.GetFullUnityVersion();
                
                //Remove GUID in parenthesis 
                return version.Substring(0, version.LastIndexOf(" ("));
            }
            
            public static bool supportedMajorVersion = true;
            public static bool supportedPatchVersion = true;
            public static bool compatibleVersion = true;
            
            private static void ParseUnityVersion(string versionString, out int major, out int minor, out int patch, out UnityVersionType type)
            {
                var match = System.Text.RegularExpressions.Regex.Match(versionString, @"^(\d+)\.(\d+)\.(\d+)");
                if (match.Success)
                {
                    major = int.Parse(match.Groups[1].Value);
                    minor = int.Parse(match.Groups[2].Value);
                    patch = int.Parse(match.Groups[3].Value);
                    
                    if (versionString.Contains("b"))
                    {
                        type = UnityVersionType.Beta;
                    }
                    else if (versionString.Contains("a"))
                    {
                        type = UnityVersionType.Alpha;
                    }
                    else
                    {
                        type = UnityVersionType.Release;
                    }
                }
                else
                {
                    throw new FormatException($"Invalid Unity version format: {versionString}.");
                }
            }
            
            public static void CheckUnityVersion()
            {
                string unityVersion = GetUnityVersion();
                
                #if !UNITY_6000_0_OR_NEWER
                compatibleVersion = false;
                #endif
                
                #if !UNITY_6000_0_OR_NEWER || UNITY_7000_OR_NEWER
                supportedMajorVersion = false;
                #endif
                
                ParseUnityVersion(unityVersion, out int major, out int minor, out int patch, out unityVersionType);

                //Get the minimum required patch version for the current Unity version (eg. 6000.1)
                if (REQUIRED_PATCH_VERSIONS.TryGetValue($"{major}.{minor}", out int minPatchVersion))
                {
                    supportedPatchVersion = patch >= minPatchVersion;
                }
                else
                {
                    //None found, current Unity version likely unknown
                    supportedPatchVersion = true;
                }

                MinRequiredUnityVersion = $"{major}.{minor}.{minPatchVersion}";
            }

            public static string apiResult;
            private static bool showPopup;

            public enum VersionStatus
            {
                UpToDate,
                Outdated
            }

            public enum QueryStatus
            {
                Fetching,
                Completed,
                Failed
            }
            public static QueryStatus queryStatus = QueryStatus.Completed;

#if SWS_DEV
            [MenuItem("SWS/Check for update")]
#endif
            public static void GetLatestVersionPopup()
            {
                CheckForUpdate(true);
            }

            private static int VersionStringToInt(string input)
            {
                //Remove all non-alphanumeric characters from version 
                input = input.Replace(".", string.Empty);
                input = input.Replace(" BETA", string.Empty);
                return int.Parse(input, System.Globalization.NumberStyles.Any);
            }

            public static void CheckForUpdate(bool showPopup = false)
            {
                //Offline
                if (Application.internetReachability == NetworkReachability.NotReachable)
                {
                    LATEST_VERSION = INSTALLED_VERSION;
                    return;
                }
                
                VersionChecking.showPopup = showPopup;

                queryStatus = QueryStatus.Fetching;

                var url = $"https://api.assetstore.unity3d.com/package/latest-version/{ASSET_ID}";

                using (System.Net.WebClient webClient = new System.Net.WebClient())
                {
                    webClient.DownloadStringCompleted += OnRetrievedAPIContent;
                    webClient.DownloadStringAsync(new System.Uri(url), apiResult);
                }
            }
            
            private class AssetStoreItem
            {
                public string name;
                public string version;
            }

            private static void OnRetrievedAPIContent(object sender, DownloadStringCompletedEventArgs e)
            {
                if (e.Error == null && !e.Cancelled)
                {
                    string result = e.Result;

                    AssetStoreItem asset = (AssetStoreItem)JsonUtility.FromJson(result, typeof(AssetStoreItem));

                    LATEST_VERSION = asset.version;
                    //LATEST_VERSION = "9.9.9";
                    
#if SWS_DEV
                    Debug.Log("<b>Asset store API</b> Update available = " + UPDATE_AVAILABLE + " (Installed:" + INSTALLED_VERSION + ") (Remote:" + LATEST_VERSION + ")");
#endif

                    queryStatus = QueryStatus.Completed;

                    if (VersionChecking.showPopup)
                    {
                        if (UPDATE_AVAILABLE)
                        {
                            if (EditorUtility.DisplayDialog(ASSET_NAME + ", version " + INSTALLED_VERSION, "An updated version is available: " + LATEST_VERSION, "Open Package Manager", "Close"))
                            {
                                OpenInPackageManager();
                            }
                        }
                        else
                        {
                            if (EditorUtility.DisplayDialog(ASSET_NAME + ", version " + INSTALLED_VERSION, "Installed version is up-to-date!", "Close")) { }
                        }
                    }
                }
                else
                {
                    Debug.LogWarning("[" + ASSET_NAME + "] Contacting update server failed: " + e.Error.Message);
                    queryStatus = QueryStatus.Failed;
                }
            }
        }
    }
}