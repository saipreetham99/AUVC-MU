// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
#if URP
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

namespace StylizedWater3.DynamicEffects
{
    public class DataRenderPass : ScriptableRenderPass
    {
        private const string profilerTag = "Water Dynamic Effects: Data";
        private static readonly ProfilingSampler profilerSampler = new ProfilingSampler(profilerTag);

        private const string LIGHTMODE_TAG = "WaterDynamicEffect";

        private float renderRange;
        private float m_renderRange;

        private int resolution;
        private static int m_resolution;
        
        private const string WaterDynamicEffectsBufferName = "_WaterDynamicEffectsBuffer";
        public static readonly int _WaterDynamicEffectsBufferID = Shader.PropertyToID(WaterDynamicEffectsBufferName);
        private const string WaterDynamicEffectsCoordsName = "_WaterDynamicEffectsCoords";
        private static readonly int _WaterDynamicEffectsCoords = Shader.PropertyToID(WaterDynamicEffectsCoordsName);
        
        private static readonly Color targetClearColor = new Color(0, 0, 0, 1);
        
        //Render pass
        FilteringSettings m_FilteringSettings;
        private readonly List<ShaderTagId> m_ShaderTagIdList = new List<ShaderTagId>()
        {
            new ShaderTagId(LIGHTMODE_TAG),
            //new ShaderTagId("UniversalForward")
        };
        
        private RendererListParams rendererListParams;
        private RendererList rendererList;

        private Vector4 rendererCoords;

        private StylizedWaterRenderFeature.DynamicEffectsSettings settings;
        
        public class RenderTargetDebugContext : RenderTargetDebugger.RenderTarget
        {
            public RenderTargetDebugContext()
            {
                this.name = "Dynamic Effects Data";
                this.description = "R: Height G: Foam B:Normal height A:Unused";
                this.textureName = WaterDynamicEffectsBufferName;
                this.propertyID = _WaterDynamicEffectsBufferID;
                this.order = 1;
            }
        }
        
        public DataRenderPass()
        {
            m_FilteringSettings = new FilteringSettings(RenderQueueRange.all, -1);
        }
        
        public void Setup(ref StylizedWaterRenderFeature.DynamicEffectsSettings settings, int targetResolution)
        {
            this.settings = settings;
            this.renderRange = settings.renderRange;
            this.resolution = PlanarProjection.CalculateResolution(settings.renderRange, settings.texelsPerUnit, 16, settings.maxResolution);
        }

        public class DynamicEffectsData : ContextItem
        {
            public TextureHandle renderTarget;

            public override void Reset()
            {
                renderTarget = TextureHandle.nullHandle;
            }
        }
        private class PassData
        {
            public RendererListHandle rendererListHandle;
            public TextureHandle renderTarget;

            public PlanarProjection planarProjection;
            public Vector4 rendererCoords;
        }
        
        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameContext)
        {
            var renderingData = frameContext.Get<UniversalRenderingData>();
            var cameraData = frameContext.Get<UniversalCameraData>();
            var lightData = frameContext.Get<UniversalLightData>();
            
            using (var builder = renderGraph.AddRasterRenderPass<PassData>(profilerTag, out var passData))
            {
                RenderTextureDescriptor renderTargetDescriptor = new RenderTextureDescriptor(resolution, resolution, GraphicsFormat.R16G16B16A16_SFloat, (int)DepthBits.None);
                
                DrawingSettings drawingSettings = CreateDrawingSettings(m_ShaderTagIdList, renderingData, cameraData, lightData, SortingCriteria.RenderQueue | SortingCriteria.SortingLayer | SortingCriteria.CommonTransparent);
                drawingSettings.perObjectData = PerObjectData.None;
                
                rendererListParams.cullingResults = renderingData.cullResults;
                rendererListParams.drawSettings = drawingSettings;
                rendererListParams.filteringSettings = m_FilteringSettings;
                
                //Render target
                passData.renderTarget = UniversalRenderer.CreateRenderGraphTexture(renderGraph, renderTargetDescriptor, WaterDynamicEffectsBufferName, true, FilterMode.Bilinear, TextureWrapMode.Clamp);
                
                //Seeing a compile error here? Update to Stylized Water v3.0.1+
                #if UNITY_EDITOR || DEVELOPMENT_BUILD
                if (RenderTargetDebugger.InspectedProperty == _WaterDynamicEffectsBufferID)
                {
                    StylizedWaterRenderFeature.DebugData debugData = frameContext.Get<StylizedWaterRenderFeature.DebugData>();
                    debugData.currentHandle = passData.renderTarget;
                }
                #endif
                
                //Set up access for the height to normal pass
                DynamicEffectsData data = frameContext.GetOrCreate<DynamicEffectsData>();
                data.renderTarget = passData.renderTarget;
                
                passData.rendererListHandle = renderGraph.CreateRendererList(rendererListParams);
                
                passData.planarProjection = new PlanarProjection
                {
                    center = cameraData.camera.transform.position,
                    offset = cameraData.camera.transform.forward * ((settings.renderRange * 0.5f) - settings.FadePercentageToLength()),
                    scale = settings.renderRange,
                    resolution = resolution
                };
                passData.planarProjection.Recalculate();
                
                passData.planarProjection.SetUV(ref passData.rendererCoords);
                
                //Set render target and bind to global property
                builder.SetRenderAttachment(passData.renderTarget, 0, AccessFlags.Write);
                //builder.CreateTransientTexture(passData.renderTarget);
                builder.SetGlobalTextureAfterPass(passData.renderTarget, _WaterDynamicEffectsBufferID);
                
                builder.UseRendererList(passData.rendererListHandle);
                builder.AllowGlobalStateModification(true);
                //builder.AllowPassCulling(false);

                builder.SetRenderFunc((PassData data, RasterGraphContext context) =>
                {
                    Execute(context, data);
                });
            }
        }

        private void Execute(RasterGraphContext context, PassData data)
        {
            var cmd = context.cmd;
            using (new ProfilingScope(cmd, profilerSampler))
            {
                cmd.ClearRenderTarget(true, true, targetClearColor);
                
                cmd.EnableShaderKeyword(ShaderParams.Keywords.DynamicEffects);

                cmd.SetViewProjectionMatrices(data.planarProjection.view, data.planarProjection.projection);
                cmd.SetViewport(data.planarProjection.viewportRect);

                cmd.SetGlobalVector(_WaterDynamicEffectsCoords, data.rendererCoords);

                cmd.DrawRendererList(data.rendererListHandle);
            }
        }
        
        public void Dispose()
        {
            Shader.DisableKeyword(ShaderParams.Keywords.DynamicEffects);
            Shader.SetGlobalVector(_WaterDynamicEffectsCoords, Vector4.zero);
        }
        
#pragma warning disable CS0672
#pragma warning disable CS0618
        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData) { }
#pragma warning restore CS0672
#pragma warning restore CS0618
    }
}
#endif