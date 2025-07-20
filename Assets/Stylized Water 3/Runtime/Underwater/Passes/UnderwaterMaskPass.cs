// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
#if URP
using UnityEngine.Rendering.Universal;

namespace StylizedWater3.UnderwaterRendering
{
    public class UnderwaterMaskPass : ScriptableRenderPass
    {
        private const string profilerTag = "Underwater Rendering: Mask";
        private static readonly ProfilingSampler profilerSampler = new ProfilingSampler(profilerTag);

        //Perfectly fine to render this at a quarter resolution
        private const int DOWNSAMPLES = 4;

        private const string UnderwaterMaskName = "_UnderwaterMask";
        private static readonly int _UnderwaterMask = Shader.PropertyToID(UnderwaterMaskName);

        private const string PASS_NAME = "Underwater Mask";
        
        public class RenderTargetDebugContext : RenderTargetDebugger.RenderTarget
        {
            public RenderTargetDebugContext()
            {
                this.name = "Underwater Mask";
                this.description = "Screen-space underwater mask";
                this.textureName = UnderwaterMaskName;
                this.propertyID = _UnderwaterMask;
            }
        }
        
        private class PassData
        {
            public TextureHandle renderTarget;
            public Mesh mesh;
            public Material material;
            public int passIndex;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            using (var builder = renderGraph.AddRasterRenderPass<PassData>("Water: Underwater Mask", out var passData))
            {
                var cameraData = frameData.Get<UniversalCameraData>();

                StylizedWaterRenderFeature.UnderwaterRenderingData data = frameData.Get<StylizedWaterRenderFeature.UnderwaterRenderingData>();

                Vector2Int resolution = new Vector2Int(cameraData.cameraTargetDescriptor.width, cameraData.cameraTargetDescriptor.height);
                
                RenderTextureDescriptor renderTargetDescriptor = new RenderTextureDescriptor(
                    resolution.x / DOWNSAMPLES, resolution.y / DOWNSAMPLES,
                    GraphicsFormat.R8_UNorm, (int)DepthBits.None);
                renderTargetDescriptor.msaaSamples = (int)MSAASamples.None;
                
                passData.renderTarget = UniversalRenderer.CreateRenderGraphTexture(renderGraph, renderTargetDescriptor, UnderwaterMaskName, true, FilterMode.Bilinear);
                passData.mesh = UnderwaterUtilities.WaterLineMesh;
                
                passData.material = data.volume.waterMaterial;
                passData.passIndex = passData.material.FindPass(PASS_NAME);

                #if UNITY_EDITOR || DEVELOPMENT_BUILD
                if (RenderTargetDebugger.InspectedProperty == _UnderwaterMask)
                {
                    StylizedWaterRenderFeature.DebugData debugData = frameData.Get<StylizedWaterRenderFeature.DebugData>();
                    debugData.currentHandle = passData.renderTarget;
                }
                #endif
                
                //builder.UseTexture(passData.renderTarget, AccessFlags.Write);
                builder.SetRenderAttachment(passData.renderTarget, 0, AccessFlags.Write);
                builder.SetGlobalTextureAfterPass(passData.renderTarget, _UnderwaterMask);
                
                builder.AllowPassCulling(false);
                builder.AllowGlobalStateModification(true);
                
                builder.SetRenderFunc((PassData data, RasterGraphContext context) =>
                {
                    Execute(context, data);
                });
            }
        }

        private void Execute(RasterGraphContext context, PassData data)
        {
            context.cmd.ClearRenderTarget(true, true, Color.black);
            context.cmd.DrawMesh(data.mesh, Matrix4x4.identity, data.material, 0, data.passIndex);
            
            //Apparently can't rely on "SetGlobalTextureAfterPass". In VR, this RT would not be set!
            //context.cmd.SetGlobalTexture(_UnderwaterMask, data.renderTarget);
        }

        public void Dispose()
        {
            
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