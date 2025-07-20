// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using UnityEngine;
using UnityEngine.Rendering;
#if URP
using UnityEngine.Rendering.Universal;
using UnityEngine.Rendering.Universal.Internal;
using UnityEngine.Rendering.RenderGraphModule;

namespace StylizedWater3
{
    public class SetupConstants : ScriptableRenderPass
    {
        private ProfilingSampler m_ProfilingSampler;
        
        private static readonly int _CausticsProjectionAvailable = Shader.PropertyToID("_CausticsProjectionAvailable");
        private static readonly int CausticsProjection = Shader.PropertyToID("CausticsProjection");
        private static readonly int _WaterSSRParams = Shader.PropertyToID("_WaterSSRParams");
        private static readonly int _WaterSSRSettings = Shader.PropertyToID("_WaterSSRSettings");

        private static VisibleLight mainLight;
        private Matrix4x4 causticsProjection;

        public SetupConstants()
        {
            //Force a unit scale, otherwise affects the projection tiling of the caustics
            causticsProjection = Matrix4x4.Scale(Vector3.one);
        }

        private StylizedWaterRenderFeature settings;
        
        public void Setup(StylizedWaterRenderFeature renderFeature)
        {
            this.settings = renderFeature;

            /*
             //Whilst required for these features, do not impose onto the render pipeline
             //User must manage the enabled state of depth texture
            if (settings.screenSpaceReflectionSettings.allow || settings.allowDirectionalCaustics)
            {
                ConfigureInput(ScriptableRenderPassInput.Depth);
            }
            */
        }
        
        private class PassData
        {
            public UniversalCameraData cameraData;
            
            public bool directionalCaustics;
            public Matrix4x4 causticsProjection;
            
            public bool ssr;
            public bool ssrSkybox;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            UniversalLightData lightData = frameData.Get<UniversalLightData>();
            UniversalCameraData cameraData = frameData.Get<UniversalCameraData>();
            
            #if UNITY_EDITOR || DEVELOPMENT_BUILD
            frameData.GetOrCreate<StylizedWaterRenderFeature.DebugData>();
            #endif
            
            using (var builder = renderGraph.AddRasterRenderPass<PassData>("Water Constants", out var passData, m_ProfilingSampler))
            {
                passData.ssr = settings.screenSpaceReflectionSettings.allow;
                passData.ssrSkybox = settings.screenSpaceReflectionSettings.reflectEverything;
                passData.directionalCaustics = settings.allowDirectionalCaustics;
            
                if (passData.directionalCaustics)
                {
                    //When no lights are visible, main light will be set to -1.
                    if (lightData.mainLightIndex > -1)
                    {
                        mainLight = lightData.visibleLights[lightData.mainLightIndex];

                        if (mainLight.lightType == LightType.Directional)
                        {
                            causticsProjection = Matrix4x4.Rotate(mainLight.light.transform.rotation);

                            passData.causticsProjection = causticsProjection.inverse;
                            passData.cameraData = cameraData;
                        }
                        else
                        {
                            passData.directionalCaustics = false;
                        }
                    }
                    else
                    {
                        passData.directionalCaustics = false;
                    }
                }
                
                //Pass should always execute
                builder.AllowPassCulling(false);

                builder.AllowGlobalStateModification(true);
                builder.SetRenderFunc((PassData data, RasterGraphContext rgContext) =>
                {
                    Execute(rgContext.cmd, data);
                });
            }
        }
        
        static void Execute(RasterCommandBuffer cmd, PassData data)
        {
            cmd.SetGlobalVector(_WaterSSRParams, new Vector4(data.ssr ? 1 : 0, data.ssrSkybox ? 1 : 0, 0));
            
            //Exposed settings not ready yet, would like to refactor the raymarching to use a step-distance, rather than a fixed number of steps.
            cmd.SetGlobalVector(_WaterSSRSettings, new Vector4(12, 0.75f, 100f, 1.0f));
            
            cmd.SetGlobalInt(_CausticsProjectionAvailable, data.directionalCaustics ? 1 : 0);
            if (data.directionalCaustics)
            {
                cmd.SetGlobalMatrix(CausticsProjection, data.causticsProjection);
                
                //Sets up the required View- -> Clip-space matrices
                NormalReconstruction.SetupProperties(cmd, data.cameraData);
            }
        }

        public override void OnCameraCleanup(CommandBuffer cmd)
        {
            //Important to disable these features, as the next camera rendering may be using a different renderer altogether
            cmd.SetGlobalInt(_CausticsProjectionAvailable, 0);
            cmd.SetGlobalVector(_WaterSSRParams, Vector4.zero);
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
#endif