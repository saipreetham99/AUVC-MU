using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

namespace StylizedWater3.UnderwaterRendering
{
    public class SetupPrePass : ScriptableRenderPass
    {
        private UnderwaterArea volume;
        
        private bool renderingEnabled;
        
        private RTHandle skyboxCubemapTextureHandle;

        public void Setup(bool enabled, UnderwaterArea volume)
        {
            this.renderingEnabled = enabled;
            this.volume = volume;
        }
        private class PassData
        {
            public bool cameraIntersecting;
            public bool submerged;
            public Vector4 parameters;

            public Vector3 cameraPosition;
            public Vector3 cameraForward;
            public float nearPlaneDistance;

            //Shader params
            public float lensOffset;
            public float waterlineWidth;
            public float waterLevel;
            public UnderwaterArea.ShadingSettings shadingSettings;
            public TextureHandle skyboxCubemap;
            public Vector4 skyboxHDRDecodeValues;
        }
        
        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            using (var builder = renderGraph.AddRasterRenderPass<PassData>("Underwater Rendering Setup", out var passData))
            {
                StylizedWaterRenderFeature.UnderwaterRenderingData underwaterRenderingData = frameData.GetOrCreate<StylizedWaterRenderFeature.UnderwaterRenderingData>();
                
                passData.cameraIntersecting = renderingEnabled;
                
                if (passData.cameraIntersecting)
                {
                    var cameraData = frameData.Get<UniversalCameraData>();
                    var camera = cameraData.camera;
                    passData.submerged = volume.CameraSubmerged(camera);
                    passData.lensOffset = volume.waterlineOffset;
                    passData.waterlineWidth = volume.waterlineThickness;
                    
                    underwaterRenderingData.enabled = passData.cameraIntersecting;
                    underwaterRenderingData.fullySubmerged = passData.submerged;
                    underwaterRenderingData.volume = volume;
                    
                    passData.shadingSettings = volume.shadingSettings;
                    passData.waterLevel = volume.CurrentWaterLevel;
                    
                    if (RenderSettings.ambientMode == AmbientMode.Skybox)
                    {
                        Texture environmentCubemap = UnderwaterShadingPass.AmbientLightOverride ? UnderwaterShadingPass.AmbientLightOverride.texture : ReflectionProbe.defaultTexture;
                        //Have to specifically create a descriptor. Enviro 3 will create a cubemap with both a color and depth format, which is invalid
                        RenderTextureDescriptor environmentCubemapDescriptor = new RenderTextureDescriptor(environmentCubemap.width, environmentCubemap.height, environmentCubemap.graphicsFormat, 0, environmentCubemap.mipmapCount);
                    
                        if (SystemInfo.IsFormatSupported(environmentCubemap.graphicsFormat, GraphicsFormatUsage.Render) == false)
                        {
                            Debug.LogWarning($"[Underwater Rendering] The skybox reflection cubemap \"{environmentCubemap.name}\" format \"{environmentCubemap.graphicsFormat}\" is reportedly not supported. " +
                                             $"This affects negatively underwater lighting. A third-party script is likely overriding this cubemap, but with an incorrect (or compressed) format.");
                        
                            //Fallback to a usable HDR format
                            environmentCubemapDescriptor.graphicsFormat = GraphicsFormat.R16G16B16A16_UNorm;
                        }

                        if (RenderingUtils.ReAllocateHandleIfNeeded(ref skyboxCubemapTextureHandle, environmentCubemapDescriptor, environmentCubemap.filterMode, environmentCubemap.wrapMode, environmentCubemap.anisoLevel, environmentCubemap.mipMapBias, environmentCubemap.name))
                        {
                            //Debug.Log($"[Underwater Rendering] Allocated skybox cubemap");
                        }

                        passData.skyboxCubemap = renderGraph.ImportTexture(skyboxCubemapTextureHandle);
                        builder.UseTexture(passData.skyboxCubemap, AccessFlags.Read);
                    
                        passData.skyboxHDRDecodeValues = UnderwaterShadingPass.AmbientLightOverride ? UnderwaterShadingPass.AmbientLightOverride.textureHDRDecodeValues : ReflectionProbe.defaultTextureHDRDecodeValues;
                    }

                    passData.cameraPosition = camera.transform.position;
                    passData.cameraForward = camera.transform.forward;
                    passData.nearPlaneDistance = camera.nearClipPlane;
                }
                else
                {
                    passData.submerged = false;
                    
                    underwaterRenderingData.fullySubmerged = false;
                    underwaterRenderingData.enabled = false;
                    underwaterRenderingData.volume = null;
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
        
        private readonly int _ClipOffset = Shader.PropertyToID("_ClipOffset");
        private readonly int _FullySubmerged = Shader.PropertyToID("_FullySubmerged");
        private readonly int _WaterLineWidth = Shader.PropertyToID("_WaterLineWidth");
        private readonly int _UnderwaterRenderingEnabled = Shader.PropertyToID("_UnderwaterRenderingEnabled");
        
        private static readonly int _UnderwaterAmbientParams = Shader.PropertyToID("_UnderwaterAmbientParams");
        private static readonly int _UnderwaterAmbientColor = Shader.PropertyToID("_UnderwaterAmbientColor");
        
        //Global values that needs to be set up again, won't survive opaque pass
        private static readonly int skyboxCubemap = Shader.PropertyToID("skyboxCubemap");
        private static readonly int skyboxCubemap_HDR = Shader.PropertyToID("skyboxCubemap_HDR");
        
        private static Vector4 ambientParams;

        void Execute(RasterCommandBuffer cmd, PassData data)
        {
            cmd.SetGlobalFloat("_WaterLevel", data.waterLevel);
            
            if (data.cameraIntersecting)
            {
                cmd.EnableShaderKeyword(ShaderParams.Keywords.UnderwaterRendering);
                cmd.SetGlobalFloat(_ClipOffset, data.lensOffset);
                cmd.SetGlobalFloat(_FullySubmerged, data.submerged ? 1 : 0);
                cmd.SetGlobalFloat(_WaterLineWidth, data.waterlineWidth);
                
                cmd.SetGlobalFloat("_StartDistance", data.shadingSettings.fogStartDistance);
                cmd.SetGlobalFloat("_UnderwaterFogDensity", data.shadingSettings.fogDensity * 0.1f);
                cmd.SetGlobalFloat("_UnderwaterSubsurfaceStrength", data.shadingSettings.translucencyStrength);
                cmd.SetGlobalFloat("_UnderwaterSubsurfaceExponent", data.shadingSettings.translucencyExponent);
                cmd.SetGlobalFloat("_UnderwaterCausticsStrength", data.shadingSettings.causticsBrightness);
                
                cmd.SetGlobalFloat("_UnderwaterFogBrightness", data.shadingSettings.fogBrightness);
                cmd.SetGlobalFloat("_UnderwaterColorAbsorption", Mathf.Pow(data.shadingSettings.colorAbsorption, 3f));
                
                cmd.SetGlobalVector("_UnderwaterHeightFogParams", new Vector4(data.shadingSettings.heightFogStart, data.shadingSettings.heightFogEnd, data.shadingSettings.heightFogDensity * 0.01f, data.shadingSettings.heightFogBrightness));
                    
                if (RenderSettings.ambientMode == AmbientMode.Skybox)
                {
                    cmd.SetGlobalTexture(skyboxCubemap, data.skyboxCubemap);
                    cmd.SetGlobalVector(skyboxCubemap_HDR, data.skyboxHDRDecodeValues);
                }
                else if (RenderSettings.ambientMode == AmbientMode.Flat)
                {
                    cmd.SetGlobalColor(_UnderwaterAmbientColor, RenderSettings.ambientLight.linear);
                }
                else //Tri-light
                {
                    cmd.SetGlobalColor(_UnderwaterAmbientColor, RenderSettings.ambientEquatorColor.linear);
                }

                ambientParams.x = Mathf.GammaToLinearSpace(RenderSettings.ambientIntensity);
                ambientParams.y = RenderSettings.ambientMode == AmbientMode.Skybox ? 1 : 0;
                cmd.SetGlobalVector(_UnderwaterAmbientParams, ambientParams);
            }
            else
            {
                cmd.DisableShaderKeyword(ShaderParams.Keywords.UnderwaterRendering);
            }
            
            cmd.SetGlobalFloat(_UnderwaterRenderingEnabled, data.cameraIntersecting ? 1 : 0);
        }
        
        public void Dispose()
        {
            skyboxCubemapTextureHandle?.Release();
        }
        
        #pragma warning disable CS0672
        #pragma warning disable CS0618
        public override void Configure(CommandBuffer cmd, RenderTextureDescriptor cameraTextureDescriptor) { }

        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData) { }
        #pragma warning restore CS0672
        #pragma warning restore CS0618
    }
}