// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule.Util;
#if URP
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

namespace StylizedWater3.UnderwaterRendering
{
    public class WaterlinePass : ScriptableRenderPass
    {
        private StylizedWaterRenderFeature.UnderwaterRenderingSettings settings;
        private Material material;

        public void Setup(StylizedWaterRenderFeature.UnderwaterRenderingSettings m_settings, StylizedWaterRenderFeature.UnderwaterSurface surface)
        {
            this.settings = m_settings;
            this.material = surface.waterlineMaterial;
        }

        public class PassData
        {
            public Mesh mesh;
            public Material material;
        }
        
        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            var resourceData = frameData.Get<UniversalResourceData>();
            
            //using (var builder = renderGraph.AddUnsafePass<PassData>("Underwater Rendering", out var passData))
            using (var builder = renderGraph.AddRasterRenderPass<PassData>("Water: Waterline", out var passData))
            {
                StylizedWaterRenderFeature.UnderwaterRenderingData data = frameData.Get<StylizedWaterRenderFeature.UnderwaterRenderingData>();

                passData.mesh = UnderwaterUtilities.WaterLineMesh;
                passData.material = material;
                
                builder.AllowPassCulling(false);
                //builder.AllowGlobalStateModification(true);
                
                builder.SetRenderAttachment(resourceData.activeColorTexture, 0, AccessFlags.Write);

                builder.SetRenderFunc((PassData data, RasterGraphContext context) =>
                {
                    Execute(context, data);
                });
            }
        }
        
        private void Execute(RasterGraphContext context, PassData data)
        {
            context.cmd.DrawMesh(data.mesh, Matrix4x4.identity, data.material, 1);
        }
        
        #pragma warning disable CS0672
        #pragma warning disable CS0618
        public override void Configure(CommandBuffer cmd, RenderTextureDescriptor cameraTextureDescriptor) { }

        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData) { }
        #pragma warning restore CS0672
        #pragma warning restore CS0618
    }

}
#endif