// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using StylizedWater3.DynamicEffects;
using UnityEditor;
using UnityEngine;

namespace StylizedWater3
{
    #if URP
    partial class MaterialUI : ShaderGUI
    {
        private bool dynamicEffectsInitialized;
        
        private bool deRenderFeaturePresent;
        private bool deRenderFeatureEnabled;
        
        partial void DrawDynamicEffectsUI()
        {
            if (!dynamicEffectsInitialized)
            {
                deRenderFeaturePresent = PipelineUtilities.GetRenderFeature<StylizedWaterRenderFeature>();
                if(deRenderFeaturePresent) deRenderFeatureEnabled = PipelineUtilities.IsRenderFeatureEnabled<StylizedWaterRenderFeature>();
                
                dynamicEffectsInitialized = true;
            }
            
            using (new EditorGUI.DisabledGroupScope(Application.isPlaying))
            {
                UI.DrawNotification(!deRenderFeaturePresent, "The Dynamic Effects extension is installed, but the render feature hasn't been setup on the default renderer", "Add", () =>
                {
                    PipelineUtilities.AddRenderFeature<StylizedWaterRenderFeature>(name:"Stylized Water 3: Dynamic Effects");
                    deRenderFeaturePresent = true;
                    deRenderFeatureEnabled = true;
                }, MessageType.Error);
            }
            if(Application.isPlaying && !deRenderFeaturePresent) EditorGUILayout.HelpBox("Exit play mode to perform this action", MessageType.Warning);
        
            UI.DrawNotification(deRenderFeaturePresent && !deRenderFeatureEnabled, "The Dynamic Effects render feature is disabled", "Enable", () => 
            { 
                PipelineUtilities.ToggleRenderFeature<StylizedWaterRenderFeature>(true);
                deRenderFeatureEnabled = true; 
            }, MessageType.Warning);
        } 
    }
    #endif
}