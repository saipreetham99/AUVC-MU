// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using System.Collections.Generic;
using UnityEngine;
using StylizedWater3.UnderwaterRendering;
#if URP
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace StylizedWater3
{
    public partial class StylizedWaterRenderFeature : ScriptableRendererFeature
    {
        [Serializable]
        public class UnderwaterRenderingSettings
        {
            public bool enable = true;
            public enum RenderMethod
            {
                RenderPass,
                SceneRenderer
            }
            [Tooltip("Specifies the render method to use for the underwater rendering.\n\n" +
                     "[Render Pass]: Renders the underwater rendering directly after all other Transparent materials. This also supports split-screen and multiple cameras." +
                     "\n\n" +
                     "[Scene Renderer]: Renders as a traditional Mesh Renderer in the scene, allowing control over the render queue. Fastest to render, recommended for VR.")]
            public RenderMethod renderMethod = RenderMethod.RenderPass;
            
            public bool ignoreSceneView;
        }
        public UnderwaterRenderingSettings underwaterRenderingSettings = new UnderwaterRenderingSettings();
        
        //Shared resources, ensures they're included in a build when the render feature is in use
        public UnderwaterResources underwaterResources;
        
        private SetupPrePass underwaterPrePass;
        private UnderwaterMaskPass maskPass;
        private UnderwaterShadingPass shadingPass;
        private WaterlinePass waterlinePass;

        public class UnderwaterSurface
        {
            public MeshRenderer meshRenderer;
            public MeshFilter meshFilter;

            public Material shadingMaterial;
            public Material waterlineMaterial;
            
            public void UpdateMaterial(Material waterMaterial, int renderingOrder)
            {
                shadingMaterial.CopyMatchingPropertiesFromMaterial(waterMaterial);
                shadingMaterial.renderQueue = waterMaterial.renderQueue + renderingOrder +1;
			
                waterlineMaterial.CopyMatchingPropertiesFromMaterial(shadingMaterial);
            }
        }
        public Dictionary<UnderwaterArea, UnderwaterSurface> underwaterSurfaces = new Dictionary<UnderwaterArea, UnderwaterSurface>();

        //Shared between passes
        public class UnderwaterRenderingData : ContextItem
        {
            public bool enabled;
            public bool fullySubmerged;

            public UnderwaterArea volume;
            
            public override void Reset()
            {
                
            }
        }

        partial void VerifyUnderwaterRendering()
        {
            if (!underwaterResources) underwaterResources = UnderwaterResources.Find();
        }

        partial void CreateUnderwaterRenderingPasses()
        {
            #if UNITY_EDITOR
            VerifyUnderwaterRendering();
            #endif
            
            underwaterPrePass = new SetupPrePass();
            underwaterPrePass.renderPassEvent = RenderPassEvent.BeforeRendering;
            
            maskPass = new UnderwaterMaskPass();
            maskPass.renderPassEvent = RenderPassEvent.BeforeRenderingTransparents;

            shadingPass = new UnderwaterShadingPass();
            shadingPass.renderPassEvent = RenderPassEvent.BeforeRenderingTransparents;

            waterlinePass = new WaterlinePass();
            waterlinePass.renderPassEvent = RenderPassEvent.AfterRenderingTransparents;
        }
        
        private bool RenderForCamera(CameraData cameraData, Camera camera)
        {
            CameraType cameraType = camera.cameraType;
            
            if (cameraType == CameraType.SceneView && underwaterRenderingSettings.ignoreSceneView) return false;

            CameraRenderType renderType = cameraData.renderType;
            
            //Camera stacking and depth-based effects is essentially non-functional.
            //All effects render twice to the screen, causing double brightness. Next to fog causing overlay objects to appear transparent
            //- Best option is to not render anything for overlay cameras
            //- Reflection probes do not capture the water line correctly
            //- Preview cameras end up rendering the effect into asset thumbnails
            if (renderType == CameraRenderType.Overlay || cameraType == CameraType.Reflection || cameraType == CameraType.Preview) return false;

            //if (cameraType != CameraType.SceneView && camera.targetTexture) return false;
            
#if UNITY_EDITOR
            //Skip if post-processing is disabled in scene-view
            if (cameraType == CameraType.SceneView && UnityEditor.SceneView.lastActiveSceneView && !UnityEditor.SceneView.lastActiveSceneView.sceneViewState.showImageEffects) return false;
#endif
            
            #if UNITY_EDITOR
            //Skip rendering if editing a prefab
            if (UnityEditor.SceneManagement.PrefabStageUtility.GetCurrentPrefabStage()) return false;
            #endif
            
            return true;
        }
        

        partial void AddUnderwaterRenderingPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            if (!underwaterRenderingSettings.enable || UnderwaterArea.Instances.Count == 0)
            {
                Shader.SetGlobalFloat("_UnderwaterRenderingEnabled", 0);
                Shader.DisableKeyword(ShaderParams.Keywords.UnderwaterRendering);

                return;
            }
            
            var camera = renderingData.cameraData.camera;

            if (underwaterRenderingSettings.renderMethod == StylizedWaterRenderFeature.UnderwaterRenderingSettings.RenderMethod.SceneRenderer)
            {
                //Hide, until a camera actually needs to render it
                foreach (var underwaterSurface in underwaterSurfaces)
                {
                    underwaterSurface.Value.meshRenderer.forceRenderingOff = true;
                }
            }
            
            UnderwaterArea area = UnderwaterArea.GetFirstIntersecting(camera);

            bool render = underwaterRenderingSettings.enable && area;
            render &= RenderForCamera(renderingData.cameraData, camera);
            
            underwaterPrePass.Setup(render, area);
            renderer.EnqueuePass(underwaterPrePass);

            if (render == false)
            {
                return;
            }

            //Debug.Log($"Rendering for: {camera.name} (intersects with {volume.name})");
            
            if (!underwaterSurfaces.TryGetValue(area, out UnderwaterSurface surface))
            {
                surface = CreateSurface(area);
            }

            if (!surface.meshRenderer)
            {
                Debug.LogError($"Underwater Surface was unintentionally deleted for \"{area.name}\"");
                
                surface = CreateSurface(area);
            }

            surface.meshRenderer.enabled = underwaterRenderingSettings.renderMethod == UnderwaterRenderingSettings.RenderMethod.SceneRenderer;
            surface.meshRenderer.forceRenderingOff = false;
                                
            //This doesn't actually appear to work with multiple cameras active!
            area.UpdateWithCamera(camera);
            
            //Copy the water material parameters to the underwater materials
            surface.UpdateMaterial(area.waterMaterial, area.renderingOrder);

            //Position at least in front of the camera to avoid being culled
            if (underwaterRenderingSettings.renderMethod == UnderwaterRenderingSettings.RenderMethod.SceneRenderer)
            {
                surface.meshRenderer.transform.position = camera.transform.position + (camera.transform.forward * 5f);
            }
            
            renderer.EnqueuePass(maskPass);

            if (underwaterRenderingSettings.renderMethod == UnderwaterRenderingSettings.RenderMethod.RenderPass)
            {
                shadingPass.Setup(underwaterRenderingSettings, surface);
                renderer.EnqueuePass(shadingPass);
                
                waterlinePass.Setup(underwaterRenderingSettings, surface);
                renderer.EnqueuePass(waterlinePass);
            }
        }

        private UnderwaterSurface CreateSurface(UnderwaterArea area)
        {
            UnderwaterSurface surface = new UnderwaterSurface();
                
            GameObject surfaceGO = new GameObject("Underwater Surface");
            surfaceGO.transform.SetParent(area.transform);
                
            surfaceGO.hideFlags = HideFlags.DontSave;
            surfaceGO.hideFlags |= HideFlags.HideInHierarchy;
                
            surface.meshRenderer = surfaceGO.AddComponent<MeshRenderer>();
            surface.meshRenderer.allowOcclusionWhenDynamic = false;
            surface.meshRenderer.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion;
            surface.meshRenderer.shadowCastingMode = ShadowCastingMode.Off;

            surface.meshFilter = surfaceGO.AddComponent<MeshFilter>();

            surface.meshFilter.sharedMesh = UnderwaterUtilities.WaterLineMesh;

            surface.shadingMaterial = new Material(underwaterResources.underwaterShader);
            surface.shadingMaterial.name = "Underwater Rendering";
                
            surface.waterlineMaterial = new Material(underwaterResources.waterlineShader);
            surface.waterlineMaterial.name = "Waterline";
                
            surface.meshRenderer.materials = new []{ surface.shadingMaterial, surface.waterlineMaterial };

            underwaterSurfaces.Remove(area, out var _);
            underwaterSurfaces.Add(area, surface);
                
            //Debug.Log($"Created Underwater Surface for area: {area.name}");

            return surface;
        }

        partial void DisposeUnderwaterRenderingPasses()
        {
            maskPass.Dispose();
            underwaterPrePass.Dispose();
            
            Shader.DisableKeyword(ShaderParams.Keywords.UnderwaterRendering);
            
            foreach (var surface in underwaterSurfaces)
            {
                if(surface.Value.meshRenderer) CoreUtils.Destroy(surface.Value.meshRenderer.gameObject);
            }
            underwaterSurfaces.Clear();
        }
    }
}
#endif