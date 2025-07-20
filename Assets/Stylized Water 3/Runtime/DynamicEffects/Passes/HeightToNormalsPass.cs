// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

namespace StylizedWater3.DynamicEffects
{
    internal class HeightToNormalsPass : ScriptableRenderPass
    {
        private const string profilerTag = "Water Dynamic Effects: Normals";
        private static readonly ProfilingSampler profilerSampler = new ProfilingSampler(profilerTag);

        //Channel the normal gradient resides in
        private const int NORMAL_CHANNEL = 2; //Blue
        
        private RTHandle renderTarget;
        private Material material;

        private int resolution;
        private int m_resolution;
        private bool mipmaps;
        private bool m_mipmaps;
        
        private static readonly string WaterDynamicEffectsNormalsName = "_WaterDynamicEffectsNormals";
        private static readonly int _WaterDynamicEffectsNormalsID = Shader.PropertyToID(WaterDynamicEffectsNormalsName);
        private static readonly int _HeightToNormalParams = Shader.PropertyToID("_HeightToNormalParams");

        public class RenderTargetDebugContext : RenderTargetDebugger.RenderTarget
        {
            public RenderTargetDebugContext()
            {
                this.name = "Dynamic Effects Normals";
                this.description = "Dynamic effects height data converted into a normal map";
                this.textureName = WaterDynamicEffectsNormalsName;
                this.propertyID = _WaterDynamicEffectsNormalsID;
                this.order = 2;
            }
        }
        
        public void Setup(int targetResolution, bool mipmapsEnabled, Shader shader)
        {
            this.resolution = targetResolution;
            this.mipmaps = mipmapsEnabled;

            if (!material && shader) material = CoreUtils.CreateEngineMaterial(shader);
        }
        
        private RenderTextureDescriptor renderTargetDescriptor;

        private class PassData
        {
            public RendererListHandle rendererListHandle;
            public TextureHandle renderTarget;
            public TextureHandle input;

            public Material material;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameContext)
        {
            using (var builder = renderGraph.AddUnsafePass<PassData>(profilerTag, out var passData))
            {
                DataRenderPass.DynamicEffectsData frameData = frameContext.Get<DataRenderPass.DynamicEffectsData>();

                if (frameData.Equals(null))
                {
                    return;
                }
                
                passData.input = frameData.renderTarget;
                
                renderTargetDescriptor = new RenderTextureDescriptor(resolution, resolution, GraphicsFormat.R8G8_UNorm, 0)
                {
                    useMipMap = mipmaps,
                    autoGenerateMips = true
                };

                passData.renderTarget = UniversalRenderer.CreateRenderGraphTexture(renderGraph, renderTargetDescriptor, WaterDynamicEffectsNormalsName, true, FilterMode.Bilinear, TextureWrapMode.Clamp);
                
                //CreateRenderGraphTexture function does not respect mipmap settings. Extract TextureDesc, set values and recreate again
                //Mipmaps are broken in DX12, they'll end up black
                if (mipmaps && SystemInfo.graphicsDeviceType != GraphicsDeviceType.Direct3D12)
                {
                    TextureDesc rgDesc = passData.renderTarget.GetDescriptor(renderGraph);
                    rgDesc.useMipMap = mipmaps;
                    rgDesc.autoGenerateMips = true;
                    passData.renderTarget = renderGraph.CreateTexture(rgDesc);
                }
                
                //Seeing a compile error here? Update to Stylized Water v3.0.1+
                #if UNITY_EDITOR || DEVELOPMENT_BUILD
                if (RenderTargetDebugger.InspectedProperty == _WaterDynamicEffectsNormalsID)
                {
                    StylizedWaterRenderFeature.DebugData debugData = frameContext.Get<StylizedWaterRenderFeature.DebugData>();
                    debugData.currentHandle = passData.renderTarget;
                }
                #endif

                passData.material = material;
                
                builder.UseTexture(passData.input, AccessFlags.Read);
                builder.UseTexture(passData.renderTarget, AccessFlags.Write);

                builder.AllowPassCulling(false);

                builder.SetGlobalTextureAfterPass(passData.renderTarget, _WaterDynamicEffectsNormalsID);
                builder.AllowGlobalStateModification(true);
                
                builder.SetRenderFunc((PassData data, UnsafeGraphContext context) =>
                {
                    Execute(context, data);
                });
            }
        }

        private static readonly Vector4 ScaleBias = new Vector4(1, 1, 0, 0);
        private void Execute(UnsafeGraphContext context, PassData data)
        {
            CommandBuffer unsafeCmd = CommandBufferHelpers.GetNativeCommandBuffer(context.cmd);
            using (new ProfilingScope(unsafeCmd, profilerSampler))
            {
                context.cmd.SetRenderTarget(data.renderTarget, 0, CubemapFace.Unknown, 0);
                //context.cmd.ClearRenderTarget(false, true, new Color(0.5f, 0.5f, 1.0f, 1.0f));
                context.cmd.SetGlobalVector(_HeightToNormalParams, new Vector4(1f, NORMAL_CHANNEL, 0, 0f));
                
                Blitter.BlitTexture(unsafeCmd, data.input, ScaleBias, data.material, 0);
                context.cmd.SetGlobalTexture(_WaterDynamicEffectsNormalsID, data.renderTarget);
            }
        }
        
        public void Dispose()
        {
            RTHandles.Release(renderTarget);
            Object.DestroyImmediate(material);
        }
        
#pragma warning disable CS0672
#pragma warning disable CS0618
        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData) { }
#pragma warning restore CS0672
#pragma warning restore CS0618
    }
}