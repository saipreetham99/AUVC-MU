// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

namespace StylizedWater3.DynamicEffects
{
    public class SetupConstants : ScriptableRenderPass
    {
        private Vector4 parameters;
        public readonly int _WaterDynamicEffectsParams = Shader.PropertyToID("_WaterDynamicEffectsParams");

        private class PassData
        {
            public Vector4 parameters;
        }
        
        public override void RecordRenderGraph(UnityEngine.Rendering.RenderGraphModule.RenderGraph renderGraph, ContextContainer frameData)
        {
            using (var builder = renderGraph.AddRasterRenderPass<PassData>("Dynamic Effects Constants", out var passData))
            {
                passData.parameters = this.parameters;
                
                //Pass should always execute
                builder.AllowPassCulling(false);
                
                builder.AllowGlobalStateModification(true);
                builder.SetRenderFunc((PassData data, RasterGraphContext rgContext) =>
                {
                    Execute(rgContext.cmd, data);
                });
            }
        }

        void Execute(RasterCommandBuffer cmd, PassData data)
        {
            cmd.EnableShaderKeyword(ShaderParams.Keywords.DynamicEffects);
            cmd.SetGlobalVector(_WaterDynamicEffectsParams, data.parameters);
        }
        
        public void Setup(ref StylizedWaterRenderFeature.DynamicEffectsSettings settings)
        {
            parameters.x = settings.enableNormals ? 1 : 0;
            parameters.y = settings.enableDisplacement ? 1 : 0;
            parameters.z = settings.renderRange;
            parameters.w = settings.FadePercentageToLength();
        }

        public override void OnCameraCleanup(CommandBuffer cmd)
        {
            cmd.DisableShaderKeyword(ShaderParams.Keywords.DynamicEffects);
            cmd.SetGlobalVector(_WaterDynamicEffectsParams, Vector4.zero);
        }

        public void Dispose()
        {
        }
#pragma warning disable CS0672
#pragma warning disable CS0618
        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData) { }
#pragma warning restore CS0672
#pragma warning restore CS0618
    }
}