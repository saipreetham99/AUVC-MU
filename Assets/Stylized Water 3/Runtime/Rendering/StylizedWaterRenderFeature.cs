// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

#if UNITY_EDITOR || DEVELOPMENT_BUILD
#define DEBUG_AVAILABLE
#endif

#if URP
using System;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.RenderGraphModule.Util;
using UnityEngine.Rendering.Universal;
using UnityEngine.Serialization;

namespace StylizedWater3
{
    [DisallowMultipleRendererFeature("Stylized Water 3")]
    public partial class StylizedWaterRenderFeature : ScriptableRendererFeature
    {
        [Tooltip("Render all transparent materials NOT on the \"Water\" layer into a screen-space texture, and use that for refraction rendering in the water." +
                 "\n\n" +
                 "If enabled, transparent materials can be submerged and refracted correctly")]
        public bool transparencyRefraction;
        
        public static StylizedWaterRenderFeature GetDefault()
        {
            return (StylizedWaterRenderFeature)PipelineUtilities.GetRenderFeature<StylizedWaterRenderFeature>();
        }
        
        [Serializable]
        public class ScreenSpaceReflectionSettings
        {
            [FormerlySerializedAs("enable")]
            [Tooltip("Allow SSR to be rendered in water materials that have it enabled." +
                     "\n\nDisable as a global performance scaling measure")]
            public bool allow = true;
            
            [FormerlySerializedAs("reflectSkybox")]
            [Tooltip("Only enable when Reflection Probes cannot be used in a realtime lighting setup. If enabled, SSR will also reflects the skybox color and geometry in front of the water." +
                     "\n\nIdeally disabled, so that Reflection Probes can be relied on for a 1:1 accurate skybox reflection.")]
            public bool reflectEverything = false;
        }
        public ScreenSpaceReflectionSettings screenSpaceReflectionSettings = new ScreenSpaceReflectionSettings();
        
        [FormerlySerializedAs("directionalCaustics")]
        [Tooltip("Pass on the main directional light's projection onto the water shader. Allows caustics to project along its direction (rather than top-down)." +
                 "\n\nThis shading operation is relative expensive, as it involves analyzing the depth texture at 4 different points")]
        public bool allowDirectionalCaustics;
        
        public HeightPrePass.Settings heightPrePassSettings = new HeightPrePass.Settings();
        #if SWS_DEV
        public TerrainHeightPrePass.Settings terrainHeightPrePassSettings = new TerrainHeightPrePass.Settings();
        #endif
        
        private SetupConstants constantsSetup;
        private HeightPrePass heightPrePass;
        private HeightQuerySystem.RenderPass heightQueryPass;
        #if SWS_DEV
        private TerrainHeightPrePass terrainHeightPrePass;
        private DistanceFieldPass distanceFieldPass;
        private RenderTransparentTexture transparentTexturePass;
        #endif

        #if DEBUG_AVAILABLE
        private DebugInspectorPass debugInspectorPass;
        #endif
        
        [SerializeField]
        public ComputeShader heightReadbackCS;
        [SerializeField]
        public Shader heightProcessingShader;

        /// <summary>
        /// Set this to true from a render pass if it requires the displacement pre-pass, despite it being disabled in the render feature settings.
        /// </summary>
        public static bool RequireHeightPrePass;

        protected bool WillExecuteHeightPrePass => RequireHeightPrePass || heightPrePassSettings.enable || HeightQuerySystem.RequiresHeightPrepass;
        
        private void OnValidate()
        {
            VerifyReferences();
        }

        private void Reset()
        {
            VerifyReferences();
        }
        
        public void VerifyReferences()
        {
            if (!heightReadbackCS)
            {
                #if UNITY_EDITOR
                //HeightSampler.cs
                string assetPath = UnityEditor.AssetDatabase.GUIDToAssetPath("768e0c28dfdbc6b429fd59518fa03f2d");

                ComputeShader cs = UnityEditor.AssetDatabase.LoadAssetAtPath<ComputeShader>(assetPath);

                if (cs)
                {
                    heightReadbackCS = cs;
                }
                #endif
            }
            if(!heightProcessingShader) heightProcessingShader = Shader.Find(ShaderParams.ShaderNames.HeightProcessor);
            
            #if SWS_DEV
            if(!terrainHeightPrePassSettings.terrainHeightVisualizationShader) terrainHeightPrePassSettings.terrainHeightVisualizationShader = Shader.Find(ShaderParams.ShaderNames.TerrainHeight);
            #endif

            VerifyUnderwaterRendering();
        }
        
        public class DebugData : ContextItem
        {
            public TextureHandle currentHandle;
            
            public override void Reset()
            {
                currentHandle = TextureHandle.nullHandle;
            }
        }
        
        public override void Create()
        {
            GraphicsDeviceType currentGraphicsAPI = SystemInfo.graphicsDeviceType;
            //https://issuetracker.unity3d.com/issues/crash-on-gfxdeviced3d12base-drawbufferscommon-when-adding-specific-custom-render-pass-feature-to-renderer
            if (currentGraphicsAPI == GraphicsDeviceType.Direct3D12 || currentGraphicsAPI == GraphicsDeviceType.XboxOneD3D12)
            {
                //Using the "BeforeRendering" point causes a fatal crash when using DX12 when allocating a RT
                defaultInjectionPoint = RenderPassEvent.BeforeRenderingShadows;
            }
            
            constantsSetup = new SetupConstants
            {
                renderPassEvent = defaultInjectionPoint
            };

            heightPrePass = new HeightPrePass
            {
                renderPassEvent = defaultInjectionPoint
            };

            heightQueryPass = new HeightQuerySystem.RenderPass()
            {
                renderPassEvent = defaultInjectionPoint
            };

            #if SWS_DEV
            terrainHeightPrePass = new TerrainHeightPrePass()
            {
                renderPassEvent = defaultInjectionPoint
            };
            
            if (transparencyRefraction)
            {
                transparentTexturePass = new RenderTransparentTexture();
                transparentTexturePass.renderPassEvent = RenderPassEvent.BeforeRenderingTransparents;
            }
            #endif
            
            CreateFlowMapPass();
            CreateDynamicEffectsPasses();
            CreateUnderwaterRenderingPasses();

            #if DEBUG_AVAILABLE
            debugInspectorPass = new DebugInspectorPass();
            debugInspectorPass.renderPassEvent = RenderPassEvent.AfterRendering;
            #endif
        }

        //Note: Actually prefer to render before transparents, but this creates a recursive RenderSingleCamera call
        //Restoring the view/projection to that of the camera also breaks VR. Required functions are internal URP code
        
        //In some cases, if no pre-passes render (depth, shadows, etc) then the projection does not get reset when rendering the opaque objects pass. Hence, things must render as early as possible.
        private static RenderPassEvent defaultInjectionPoint = RenderPassEvent.BeforeRendering;
        
        //Dynamic Effects
        partial void CreateDynamicEffectsPasses();
        partial void AddDynamicEffectsPasses(ScriptableRenderer renderer, ref RenderingData renderingData);
        partial void DisposeDynamicEffectsPasses();
        
        partial void VerifyUnderwaterRendering();
        partial void CreateUnderwaterRenderingPasses();
        partial void AddUnderwaterRenderingPasses(ScriptableRenderer renderer, ref RenderingData renderingData);
        partial void DisposeUnderwaterRenderingPasses();
        
        partial void CreateFlowMapPass();
        partial void AddFlowMapPass(ScriptableRenderer renderer, ref RenderingData renderingData);
        partial void DisposeFlowMapPass();

        private bool IsInvalidContext(CameraType cameraType, CameraRenderType cameraRenderType)
        {
            //Skip for any special use camera's (except scene view camera)
            if (cameraType != CameraType.SceneView && (cameraType == CameraType.Preview || hideFlags != HideFlags.None))
            {
                return true;
            }

            //Skip overlay cameras
            if (cameraRenderType == CameraRenderType.Overlay)
            {
                return true;
            }

            return false;
        }
        
        private bool isMainCamera(Camera camera)
        {
            //Skip for any special use camera's (except scene view camera)
            return (camera.cameraType == CameraType.SceneView || camera.CompareTag("MainCamera"));
        }

        public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            var currentCam = renderingData.cameraData.camera;
            
            if(IsInvalidContext(currentCam.cameraType, renderingData.cameraData.renderType)) return;
     
            constantsSetup.Setup(this);
            renderer.EnqueuePass(constantsSetup);

            #if SWS_DEV
            if (terrainHeightPrePassSettings.enable)
            {
                terrainHeightPrePass.Setup(terrainHeightPrePassSettings);
                renderer.EnqueuePass(terrainHeightPrePass);
            }
            
            if (transparencyRefraction)
            {
                renderer.EnqueuePass(transparentTexturePass);
            }
            #endif

            //Do not execute for reflection probe captures
            if (currentCam.cameraType != CameraType.Reflection)
            {
                AddFlowMapPass(renderer, ref renderingData);
                AddDynamicEffectsPasses(renderer, ref renderingData);

                //Do not execute for the scene-view camera in play-mode. Even if the tab is not active, it would render around it instead of the main camera
                var skipHeightPrePass = Application.isPlaying && heightPrePassSettings.disableInSceneView && currentCam.cameraType == CameraType.SceneView;
  
                if (WillExecuteHeightPrePass && skipHeightPrePass == false)
                {
                    heightPrePass.Setup(heightPrePassSettings);
                    renderer.EnqueuePass(heightPrePass);

                    if (HeightQuerySystem.QueryCount > 0)
                    {
                        heightQueryPass.Setup(this, heightReadbackCS);
                        renderer.EnqueuePass(heightQueryPass);
                    }
                }
                else
                {
                    Shader.SetGlobalInt(HeightPrePass._WaterHeightPrePassAvailable, 0);
                }
            }

            AddUnderwaterRenderingPasses(renderer, ref renderingData);
            
            #if DEBUG_AVAILABLE
            if (RenderTargetDebugger.InspectedProperty > 0)
            {
                renderer.EnqueuePass(debugInspectorPass);
            }
            #endif
        }

        protected override void Dispose(bool disposing)
        {
            constantsSetup.Dispose();
            heightPrePass.Dispose();
            heightQueryPass.Dispose();
            #if SWS_DEV
            terrainHeightPrePass.Dispose();
            #endif
            
            DisposeFlowMapPass();
            DisposeDynamicEffectsPasses();
            DisposeUnderwaterRenderingPasses();
        }

        #if DEBUG_AVAILABLE
        private class DebugInspectorPass : ScriptableRenderPass
        {
            public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
            {
                DebugData debugData = frameData.Get<DebugData>();
                
                //Whichever pass's render target PropertyID matches the selected one in the inspector window get assigned as the 'currentHandle'
                if (debugData.currentHandle.IsValid())
                {
                    var destinationDesc = renderGraph.GetTextureDesc(debugData.currentHandle);
                    destinationDesc.clearBuffer = false;

                    RenderTextureDescriptor rtDsc = new RenderTextureDescriptor
                    {
                        width = destinationDesc.width,
                        height = destinationDesc.height,
                        //If you're seeing an error here you are not using a compatible Unity version!
                        graphicsFormat = destinationDesc.colorFormat,
                        #if UNITY_6000_1_OR_NEWER
                        colorFormat = RenderTextureFormat.Default,
                        #endif
                        volumeDepth = 1,
                        dimension = destinationDesc.dimension,
                        useMipMap = destinationDesc.useMipMap,
                        msaaSamples = 1
                    };

                    TextureDesc textureDesc = debugData.currentHandle.GetDescriptor(renderGraph);
                    RenderingUtils.ReAllocateHandleIfNeeded(ref RenderTargetDebugger.CurrentRT, rtDsc, textureDesc.filterMode, textureDesc.wrapMode, textureDesc.anisoLevel, textureDesc.mipMapBias, textureDesc.name);
                    
                    TextureHandle destination = renderGraph.ImportTexture(RenderTargetDebugger.CurrentRT);

                    if (destination.IsValid() == false)
                    {
                        throw new Exception("Failed to generate debugger texture");
                    }
                    else
                    {
                        var cameraData = frameData.Get<UniversalCameraData>();
                        RenderTargetDebugger.CurrentCameraName = cameraData.camera.name;
                        
                        //Copy TextureHandle into persistent RT
                        renderGraph.AddCopyPass(debugData.currentHandle, destination, passName: "Water Debug");
                    }
                }
                else
                {
                    RenderTargetDebugger.CurrentRT = null;
                    RenderTargetDebugger.CurrentCameraName = string.Empty;
                }
            }

#pragma warning disable CS0672
#pragma warning disable CS0618
            public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData) { }
#pragma warning restore CS0672
#pragma warning restore CS0618
        }
        #endif //DEBUG_AVAILABLE
    }
}
#endif