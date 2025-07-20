// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using UnityEngine;
using StylizedWater3.DynamicEffects;
using UnityEngine.Serialization;
#if URP
using UnityEngine.Rendering.Universal;

namespace StylizedWater3
{
    public partial class StylizedWaterRenderFeature : ScriptableRendererFeature
    {
        [Serializable]
        public class DynamicEffectsSettings
        {
            public const int MIN_RESOLUTION = 64;

            public bool enabled = true;
            
            [Min(10f)]
            [Tooltip("Rendering is performed in an area of this size away from the camera. Longer ranges provide distant visibility, but thin out the rendering resolution.")]
            public float renderRange = 500f;
            [Range(0f, 50f)]
            [Tooltip("At the edge of the rendering area, effects fade out by this percentage (of the total area size). This is to avoid them abruptly cutting off")]
            public float fadePercentage = 10f;

            [Range(1, 32)]
            [Tooltip("Render range * Texels per unit = target resolution." +
                     "\n\nIf you were to imagine a grid, this value defines how many cells fit in one unit" +
                     "\n\n" +
                     "Lowering this value will lower the render resolution, at the cost of fine details (such as ripples)")]
            public int texelsPerUnit = 8;
            [Min(MIN_RESOLUTION)]
            [Tooltip("Given the other parameters, cap the maximum render resolution to this")]
            public int maxResolution = 4096;

            [Space]

            public bool enableDisplacement = true;
            [Tooltip("From the created displacement, create a new normal map. This is vital for lighting/shading." +
                     "\n\n" +
                     "If targeting a simple lighting setup, you can disable this")]
            public bool enableNormals = true;
            [Tooltip("Render normals at half resolution. This will mainly affect how effects influence the water's reflections")]
            public bool halfResolutionNormals;
            [Tooltip("Mipmaps for render targets will be enabled. At the cost of 33% additional memory a lower resolution texture will be sampled in the distance")]
            public bool normalMipmaps = true;

            [Space]
            
            [Tooltip("Do not execute this render feature for the scene-view camera. Helps to inspect the world while everything is rendering from the main camera's perspective")]
            public bool ignoreSceneView;

            /// <summary>
            /// Retrieve the settings objects from the current renderer. This may be used to alter settings at runtime.
            /// </summary>
            /// <returns></returns>
            /// <exception cref="Exception">Render feature not present</exception>
            public static DynamicEffectsSettings GetCurrent()
            {
                var renderFeature = GetDefault();

                if (!renderFeature)
                {
                    throw new Exception("Unable to get the current settings instance, render feature not found on the current renderer");
                }

                return renderFeature.dynamicEffectsSettings;
            }

            public float FadePercentageToLength()
            {
                return PlanarProjection.FadePercentageToLength(renderRange, fadePercentage);
            }
        }
        
        public DynamicEffectsSettings dynamicEffectsSettings = new DynamicEffectsSettings();

        private DynamicEffects.SetupConstants constantsPass;
        private DataRenderPass dataRenderPass;
        private HeightToNormalsPass normalsPass;

        partial void CreateDynamicEffectsPasses()
        {
            constantsPass ??= new DynamicEffects.SetupConstants();
            dataRenderPass ??= new DataRenderPass();
            normalsPass ??= new HeightToNormalsPass();

            constantsPass.renderPassEvent = defaultInjectionPoint;
            dataRenderPass.renderPassEvent = defaultInjectionPoint;
            normalsPass.renderPassEvent = defaultInjectionPoint;
        }

        partial void AddDynamicEffectsPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            if (DynamicEffect.InstanceCount <= 0) return;
            
            var currentCam = renderingData.cameraData.camera;
            
            //Skip for any special use camera's (except scene view camera)
            if (currentCam.cameraType != CameraType.SceneView && (currentCam.cameraType == CameraType.Reflection || currentCam.cameraType == CameraType.Preview || currentCam.hideFlags != HideFlags.None)) return;

            //Skip overlay cameras
            if (renderingData.cameraData.renderType == CameraRenderType.Overlay) return;
            
            #if UNITY_EDITOR
            if (dynamicEffectsSettings.ignoreSceneView && currentCam.cameraType == CameraType.SceneView) return;
            #endif

            int resolution = PlanarProjection.CalculateResolution(dynamicEffectsSettings.renderRange, dynamicEffectsSettings.texelsPerUnit, StylizedWaterRenderFeature.DynamicEffectsSettings.MIN_RESOLUTION, dynamicEffectsSettings.maxResolution);

            if (!dynamicEffectsSettings.enabled)
            {
                Shader.DisableKeyword(ShaderParams.Keywords.DynamicEffects);
                return;
            }
            
            constantsPass.Setup(ref dynamicEffectsSettings);
            renderer.EnqueuePass(constantsPass);
            
            dataRenderPass.Setup(ref dynamicEffectsSettings, resolution);
            renderer.EnqueuePass(dataRenderPass);

            if (dynamicEffectsSettings.enableNormals)
            {
                #if UNITY_EDITOR || DEVELOPMENT_BUILD
                if (!heightProcessingShader)
                {
                    heightProcessingShader = Shader.Find("Hidden/StylizedWater3/HeightProcessor");
                    
                    if(!heightProcessingShader) Debug.LogError("[Stylized Water 3 Dynamic Effects] A shader is missing from the render feature, causing rendering to fail. Check the inspector", this);
                }
                #endif
                
                normalsPass.Setup(resolution / (dynamicEffectsSettings.halfResolutionNormals ? 2 : 1),  dynamicEffectsSettings.normalMipmaps, heightProcessingShader);
                renderer.EnqueuePass(normalsPass);
            }
        }

        partial void DisposeDynamicEffectsPasses()
        {
            constantsPass?.Dispose();
            dataRenderPass?.Dispose();
            normalsPass?.Dispose();
        }
    }
}
#else
#error Dynamic Effects extension is imported without either the "Stylized Water 3" asset or the correct "Universal Render Pipeline" version installed. Will not be functional until these are both installed and set up.
#endif