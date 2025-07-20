using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEditor.PackageManager.Requests;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using PackageInfo = UnityEditor.PackageManager.PackageInfo;

namespace StylizedWater3
{
    //Verifies any and all project settings and states
    public static class Installer
    {
        public class SetupItem
        {
            public MessageType state = MessageType.None;
            public string name;
            public string description;
            public string actionName;
            
            public Action action;

            public SetupItem(string name)
            {
                this.name = name;
            }
            
            public SetupItem(string name, MessageType state)
            {
                this.name = name;
                this.state = state;
            }
            

            public void ExecuteAction()
            {
                action.Invoke();
                action = null;

                if (state == MessageType.Error)
                {
                    m_errorCount--;
                }
                else if (state == MessageType.Warning)
                {
                    m_warningCount--;
                }

                state = MessageType.None;
                
                Installer.Initialize();
            }
        }
        
        public static List<SetupItem> SetupItems = new List<SetupItem>();

        public static int ErrorCount
        {
            get => m_errorCount;
        }
        private static int m_errorCount;
        public static int WarningCount
        {
            get => m_warningCount;
        }
        private static int m_warningCount;
        
        public static bool HasError => ErrorCount > 0;

        private static void AddItem(SetupItem item)
        {
            if (item.state == MessageType.Error) m_errorCount++;
            else if (item.state == MessageType.Warning) m_warningCount++;
            
            SetupItems.Add(item);
        }
        
        public static void Initialize()
        {
            SetupItems.Clear();
            m_errorCount = 0;
            m_warningCount = 0;

            SetupItem unityVersion = new SetupItem(AssetInfo.VersionChecking.GetUnityVersion());
            {
                AssetInfo.VersionChecking.CheckUnityVersion();

                unityVersion.state = MessageType.None;
                unityVersion.description = $"Likely compatible and supported Unity version";
                
                
                //Too old
                if (AssetInfo.VersionChecking.compatibleVersion == false || AssetInfo.VersionChecking.supportedPatchVersion == false)
                {
                    unityVersion.state = MessageType.Error;
                    unityVersion.description = $"This version of Unity is not compatible and is not subject to support. Update to at least <b>{AssetInfo.VersionChecking.MinRequiredUnityVersion}</b>. Errors and issues need to be resolved locally";
                }
                else
                {
                    //Too broken
                    if (AssetInfo.VersionChecking.unityVersionType != AssetInfo.VersionChecking.UnityVersionType.Release)
                    {
                        unityVersion.state = MessageType.Warning;
                        unityVersion.description = "Alpha/preview versions of Unity are not supported. Shader/script errors or warnings may occur depending on which weekly-version you are using." +
                                                   "\n\n" +
                                                   "After the release version becomes available, compatibility will be verified and an update may follow with the necessary fixes/changes.";
                    }

                    //Too new
                    if (AssetInfo.VersionChecking.supportedMajorVersion == false)
                    {
                        unityVersion.state = MessageType.Warning;
                        unityVersion.description = $"This version of Unity is not supported. An upgrade version of this asset that does support this may be available. See the documentation for up-to-date information.";
                    }
                }
            }
            AddItem(unityVersion);

            AssetInfo.VersionChecking.CheckForUpdate();
            SetupItem assetVersion = new SetupItem($"Asset version ({AssetInfo.INSTALLED_VERSION})");
            {
                //Testing
                //AssetInfo.VersionChecking.compatibleVersion = false;
                //AssetInfo.VersionChecking.alphaVersion = true;
                //AssetInfo.VersionChecking.UPDATE_AVAILABLE = false;
                
                if (AssetInfo.VersionChecking.UPDATE_AVAILABLE)
                {
                    assetVersion.state = MessageType.Info;
                    assetVersion.description = $"An updated version is available (v{AssetInfo.VersionChecking.LATEST_VERSION})" +
                        "\n\n" +
                        "Asset can be updated through the Package Manager. Please update any extensions as well!";

                    assetVersion.actionName = "Open Package Manager";
                    assetVersion.action = AssetInfo.OpenInPackageManager;
                }
                else
                {
                    assetVersion.state = MessageType.None;
                    assetVersion.description = "Installed version is the latest";
                }
            }
            AddItem(assetVersion);

            SetupItem graphicsAPI = new SetupItem($"Graphics API ({PlayerSettings.GetGraphicsAPIs(EditorUserBuildSettings.activeBuildTarget)[0].ToString()})");
            {
                graphicsAPI.state = MessageType.None;
                graphicsAPI.description = $"Compatible";
            }
            AddItem(graphicsAPI);
            
            
            SetupItem colorSpace = new SetupItem($"Color space ({PlayerSettings.colorSpace.ToString()})");
            {
                if (PlayerSettings.colorSpace == ColorSpace.Linear)
                {
                    colorSpace.state = MessageType.None;
                    colorSpace.description = $"Linear";
                }
                else
                {
                    colorSpace.state = MessageType.Warning;
                    colorSpace.description = $"All content is authored for the Linear color space, water colors will not look as advertised.";
                }
            }
            AddItem(colorSpace);

            /*
            //Also counts non-script related errors!
            SetupItem compileErrors = new SetupItem("Script compilation");
            {
                ConsoleWindowUtility.GetConsoleLogCounts(out var errors, out var _, out var _);

                if (errors > 0)
                {
                    compileErrors.state = MessageType.Error;
                    compileErrors.description = "Your project has one or more script compilation errors. These can prevent the water shader from working, or components from being available!";
                }
                else
                {
                    compileErrors.state = MessageType.None;
                    compileErrors.description = "Project is free of script compilation errors";
                }
                AddItem(compileErrors);
            }
            */

            SetupItem shaderCompiler = new SetupItem("Shader Compiler");
            {
                Shader defaultShader = ShaderConfigurator.GetDefaultShader();
                var shaderCompiled = defaultShader != null;
                shaderCompiler.state = shaderCompiled ? MessageType.Error : MessageType.None;
                
                //Shader object created
                if (shaderCompiled)
                {
                    shaderCompiler.state = MessageType.None;
                    shaderCompiler.description = "Shader compiled without any errors";

                    ShaderMessage[] shaderErrors = ShaderConfigurator.GetErrorMessages(defaultShader);

                    //Compiled with errors
                    if (shaderErrors != null && shaderErrors.Length > 0)
                    {
                        shaderCompiler.state = MessageType.Error;
                        shaderCompiler.description = "Water shader has one or more critical errors:\n";

                        for (int i = 0; i < shaderErrors.Length; i++)
                        {
                            shaderCompiler.description += "\n• " + $"{shaderErrors[i].message} (line:{shaderErrors[i].line})";
                        }

                        shaderCompiler.description += "\n\nThese messages may provide a clue as to what went wrong";
                        
                        shaderCompiler.actionName = "Try recompiling";
                        shaderCompiler.action = () =>
                        {
                            string defaultShaderPath = AssetDatabase.GUIDToAssetPath(ShaderConfigurator.DEFAULT_SHADER_GUID);
                            AssetDatabase.ImportAsset(defaultShaderPath);
                        };
                    }
                    //Success
                    else
                    {
                        WaterShaderImporter importer = WaterShaderImporter.GetForShader(defaultShader);
                        bool requiresRecompile = importer.RequiresRecompilation(out var recompileMessage);

                        if (requiresRecompile)
                        {
                            shaderCompiler.state = MessageType.Warning;
                            shaderCompiler.description = "Water shader needs to be recompiled:" +
                                                         "\n" +
                                                         recompileMessage +
                                                         "\n";

                            shaderCompiler.actionName = "Recompile";
                            shaderCompiler.action = () =>
                            {
                                importer.Reimport();
                            };
                        }

                        //Shader is valid, but also check if any water meshes have a pink shader
                        {
                            Shader errorShader = Shader.Find("Hidden/InternalErrorShader");
                            List<Material> brokenWaterMaterials = new List<Material>();
                            
                            foreach (WaterObject obj in WaterObject.Instances)
                            {
                                if(obj.material == null) continue;

                                if (obj.material.shader == null)
                                {
                                    brokenWaterMaterials.Add(obj.material);
                                }
                                else if (obj.material.shader == errorShader)
                                {
                                    brokenWaterMaterials.Add(obj.material);
                                }
                            }

                            //Remove duplicates
                            brokenWaterMaterials = brokenWaterMaterials.Distinct().ToList();

                            if (brokenWaterMaterials.Count > 0)
                            {
                                SetupItem waterObjectShaderError = new SetupItem("Water Objects with invalid material");
                                waterObjectShaderError.state = MessageType.Error;
                                waterObjectShaderError.description = "One or more water objects in the scene have an invalid material and likely render as pink:";

                                for (int i = 0; i < brokenWaterMaterials.Count; i++)
                                {
                                    waterObjectShaderError.description += $"\n• {brokenWaterMaterials[i].name}";
                                }

                                waterObjectShaderError.description += "\n\nIt could be that the shader wasn't able to compile at some point and Unity assigned these materials the error-shader.";

                                waterObjectShaderError.actionName = "Fix";
                                waterObjectShaderError.action = () =>
                                {
                                    for (int i = 0; i < brokenWaterMaterials.Count; i++)
                                    {
                                        brokenWaterMaterials[i].shader = defaultShader;
                                        
                                        EditorUtility.SetDirty(brokenWaterMaterials[i]);
                                    }
                                };

                                AddItem(waterObjectShaderError);
                            }
                        }
                    }
                }
                else
                {
                    shaderCompiler.state = MessageType.Error;
                    shaderCompiler.description = "Shader failed to compile. Please ensure that there are no unresolved scripting compilation errors in the project." +
                                                 "\n\n" +
                                                 "After resolving them, re-import this asset";
                }
                
                AddItem(shaderCompiler);
            }
            
            SetupItem renderFeature = new SetupItem("Render Feature");
            {
                PipelineUtilities.RenderFeatureMissing<StylizedWaterRenderFeature>(out ScriptableRendererData[] renderers);
                    
                if (renderers.Length > 0)
                {
                    renderFeature.state = MessageType.Error;
                    renderFeature.description = "The Stylized Water render feature hasn't been added to these renderers:";

                    for (int i = 0; i < renderers.Length; i++)
                    {
                        renderFeature.description += $"\n• {renderers[i].name}";
                    }
                    
                    renderFeature.description += "\n\nThe following functionality will be unavailable if they are active:\n" +
                                                 "\n• Directional caustics" +
                                                 "\n• Screen-space Reflections" +
                                                 "\n• Water Decals" +
                                                 "\n• Height query system" +
                                                 "\n• Dynamic Effects (if installed)" +
                                                 "\n• Underwater Rendering (if installed)";

                    renderFeature.description += "\n\nFor some use cases this is intentional and this warning can be ignored";
                    
                    renderFeature.actionName = "Add to renderers";
                    renderFeature.action = () =>
                    {
                        for (int i = 0; i < renderers.Length; i++)
                        {
                            PipelineUtilities.AddRenderFeature<StylizedWaterRenderFeature>(renderers[i]);
                        }
                    };
                }
                else
                {
                    renderFeature.state = MessageType.None;
                    renderFeature.description = "Render feature has been set up on all renderers";

                    SetupItem renderGraph = new SetupItem("Render Graph", PipelineUtilities.RenderGraphEnabled() ? MessageType.None : MessageType.Error);
                    {
                        if (renderGraph.state == MessageType.Error)
                        {
                            renderGraph.description = "Disabled (Backwards Compatibility mode). The render feature (and its functionality) will be unavailable!";
                            renderGraph.actionName = "Enable";
                            renderGraph.action = () =>
                            {
                                RenderGraphSettings settings = GraphicsSettings.GetRenderPipelineSettings<RenderGraphSettings>();
                                settings.enableRenderCompatibilityMode = false;
                            };
                        }
                        else
                        {
                            renderGraph.state = MessageType.None;
                            renderGraph.description = "Render Graph is enabled";
                        }

                        AddItem(renderGraph);
                    }
                    
                    SetupItem heightPrePass = new SetupItem("Height Pre-Pass");
                    {
                        StylizedWaterRenderFeature currentRenderFeature = StylizedWaterRenderFeature.GetDefault();

                        if (currentRenderFeature.heightPrePassSettings.enable)
                        {
                            heightPrePass.state = MessageType.None;
                            heightPrePass.description = "Enabled on the default renderer";
                        }
                        else
                        {
                            heightPrePass.state = MessageType.Warning;
                            heightPrePass.description = "Disabled on the default renderer. This makes the following functionality unavailable:" + 
                                                        "\n • Water Decals" +
                                                        "\n • GPU-based height readback";

                            heightPrePass.actionName = "Enable";
                            heightPrePass.action = () =>
                            {
                                currentRenderFeature = StylizedWaterRenderFeature.GetDefault();
                                currentRenderFeature.heightPrePassSettings.enable = true;
                                
                                EditorUtility.SetDirty(currentRenderFeature);
                            };
                        }
                        
                        AddItem(heightPrePass);
                    }
                }

                AddItem(renderFeature);
            }
            
            SetupItem depthTexture = new SetupItem("Depth texture");
            {
                depthTexture.state = PipelineUtilities.IsDepthTextureOptionDisabledAnywhere(out var depthLessRenderers) ? MessageType.Warning : MessageType.None;

                if (depthTexture.state == MessageType.Warning)
                {
                    depthTexture.description = "Depth texture option is disabled these renderers:\n";

                    for (int i = 0; i < depthLessRenderers.Count; i++)
                    {
                        depthTexture.description += "• " + depthLessRenderers[i].name + "\n";
                    } 
                    depthTexture.description += "\nThis will cause depth-based effects (when enabled) such as underwater fog, caustics or refraction not to render on certain quality levels or platforms. On Android the water will turn invisible!";
                    
                    depthTexture.actionName = "Enable";
                    depthTexture.action = () =>
                    {
                        PipelineUtilities.SetDepthTextureOnAllAssets(true);
                    };
                }
                else
                {
                    depthTexture.description = "Enabled on all renderers";
                }
                AddItem(depthTexture);
            }
            
            SetupItem opaqueTexture = new SetupItem("Refraction support");
            {
                opaqueTexture.state = PipelineUtilities.IsOpaqueTextureOptionDisabledAnywhere(out var opaqueLessRenderers) ? MessageType.Warning : MessageType.None;

                if (opaqueTexture.state == MessageType.Warning)
                {
                    opaqueTexture.description = "Opaque texture option is disabled these renderers:\n";

                    for (int i = 0; i < opaqueLessRenderers.Count; i++)
                    {
                        opaqueTexture.description += "• " + opaqueLessRenderers[i].name + "\n";
                    } 
                    opaqueTexture.description += "\nThis will cause water materials with Refraction enabled to have a gray tint and not show any color of submerged geometry!";
                    
                    opaqueTexture.actionName = "Enable";
                    opaqueTexture.action = () =>
                    {
                        PipelineUtilities.SetOpaqueTextureOnAllAssets(true);
                    };
                }
                else
                {
                    opaqueTexture.description = "Opaque texture option is enabled on all renderers";

                    SetupItem opaqueDownsampled = new SetupItem("Opaque texture resolution");
                    {
                        opaqueDownsampled.state = PipelineUtilities.IsOpaqueDownSampled(out var downsampledRenderers) ? MessageType.Warning : MessageType.None;

                        if (opaqueDownsampled.state == MessageType.Warning)
                        {
                            opaqueDownsampled.description = "Opaque texture resolution is halved on these renderers:\n";
                            for (int i = 0; i < downsampledRenderers.Count; i++)
                            {
                                opaqueDownsampled.description += "• " + downsampledRenderers[i].name + "\n";
                            } 
                            opaqueDownsampled.description += "\nThis will cause water materials with Refraction enabled to appear blurry!";
                            opaqueDownsampled.description += "\n\nUnderwater rendering will also appear blurred with visible outlines around geometry!";

                            opaqueDownsampled.actionName = "Switch to full resolution";
                            opaqueDownsampled.action = () =>
                            {
                                PipelineUtilities.DisableOpaqueDownsampling(downsampledRenderers);
                            };
                        }
                        else
                        {
                            opaqueDownsampled.description = "Opaque texture is rendering at full resolution";
                        }
                        
                        AddItem(opaqueDownsampled);
                    }
                }
                AddItem(opaqueTexture);
            }

            SetupItem depthMode = new SetupItem("Depth mode");
            {
                bool hasIncorrectRenderers = PipelineUtilities.IsDepthAfterTransparents(out List<UniversalRendererData> renderers);

                if (hasIncorrectRenderers)
                {
                    depthMode.state = MessageType.Warning;
                    depthMode.description = "The depth texture is configured to render AFTER transparent materials on these renderers:";

                    for (int i = 0; i < renderers.Count; i++)
                    {
                        depthMode.description += $"\n• {renderers[i].name}";
                    }

                    depthMode.description += "\n\nThis can cause the water to appear semi-transparent, or the depth information to lag behind one frame.";
                    
                    depthMode.actionName = "Set to render before";
                    depthMode.action = () =>
                    {
                        for (int i = 0; i < renderers.Count; i++)
                        {
                            renderers[i].copyDepthMode = CopyDepthMode.AfterOpaques;
                            EditorUtility.SetDirty(renderers[i]);
                        }
                    };
                }
                else
                {
                    depthMode.state = MessageType.None;
                    depthMode.description = "The depth texture renders before transparent materials do.";
                }
                AddItem(depthMode);
            }
            
            if (StylizedWaterEditor.DynamicEffectsInstalled())
            {
                SetupItem splinesPackage = new SetupItem("Splines Package");

                var splinesInstalled = true;
                
                #if !SPLINES
                splinesInstalled = false;
                #endif

                if (splinesInstalled)
                {
                    splinesPackage.state = MessageType.None;
                    splinesPackage.description = "Installed";
                }
                else
                {
                    splinesPackage.state = MessageType.Warning;
                    splinesPackage.description = "Splines package not installed. Some functionality won't be available.";

                    splinesPackage.actionName = "Install";
                    splinesPackage.action = () =>
                    {
                        InstallPackage("com.unity.splines");
                    };
                }
                AddItem(splinesPackage);
            }
            
            SetupItem reflectionProbes = new SetupItem("Reflection Probes");
            {
                if (QualitySettings.realtimeReflectionProbes)
                {
                    reflectionProbes.state = MessageType.None;
                    reflectionProbes.description = "Realtime reflection probes are supported.";
                }
                else
                {
                    reflectionProbes.state = MessageType.Warning;
                    reflectionProbes.description = "Realtime reflection probes are disabled in Quality Settings. Water reflections may appear black in demo scenes.";
                    reflectionProbes.actionName = "Enable";
                    reflectionProbes.action = () =>
                    {
                        QualitySettings.realtimeReflectionProbes = true;
                    };
                }

                AddItem(reflectionProbes);
            }

            SetupItem asyncShaderCompilation = new SetupItem("Asynchronous Shader Compilation");
            {
                if (EditorSettings.asyncShaderCompilation)
                {
                    asyncShaderCompilation.state = MessageType.None;
                    asyncShaderCompilation.description = "Enabled in project settings";
                }
                else
                {
                    asyncShaderCompilation.state = MessageType.Error;
                    asyncShaderCompilation.description = "Disabled in project settings. This will cause the editor to crash when attempting to compile the water shader";
                    
                    asyncShaderCompilation.actionName = "Enable";
                    asyncShaderCompilation.action = () =>
                    {
                        EditorSettings.asyncShaderCompilation = true;
                    };
                }
                
                AddItem(asyncShaderCompilation);
            }

            if (WaterObject.Instances.Count > 0)
            {
                SetupItem waterLayer = new SetupItem("Objects on the \"Water\" layer");
                {
                    int waterLayerIndex = LayerMask.NameToLayer("Water");
                    
                    List<WaterObject> invalidObjects = new List<WaterObject>();
                    foreach (var obj in WaterObject.Instances)
                    {
                        if (obj.gameObject.layer != waterLayerIndex)
                        {
                            invalidObjects.Add(obj);
                        }
                    }
                    
                    var invalidObjCount = invalidObjects.Count;
                    if (invalidObjCount > 0)
                    {
                        waterLayer.state = MessageType.Warning;
                        waterLayer.description = $"{invalidObjCount} water object(s) are not on the \"Water\" layer:";

                        foreach (var obj in invalidObjects)
                        {
                            waterLayer.description += $"\n• {obj.name}";
                        }
                        
                        waterLayer.description += "\n\n This will result in them not being accessible for height queries";
                        
                        waterLayer.actionName = "Fix";
                        waterLayer.action = () =>
                        {
                            foreach (var obj in invalidObjects)
                            {
                                obj.gameObject.layer = waterLayerIndex;
                                
                                EditorUtility.SetDirty(obj);
                            }
                        };
                    }
                    else
                    {
                        waterLayer.state = MessageType.None;
                        waterLayer.description = "All water objects in the scene are on the Water layer";
                    }

                    AddItem(waterLayer);
                }
            }

            if (SceneView.lastActiveSceneView)
            {
                SetupItem sceneViewAnimations = new SetupItem("Scene view animations");
                
                if (SceneView.lastActiveSceneView.sceneViewState.alwaysRefreshEnabled == false)
                {
                    sceneViewAnimations.state = MessageType.Warning;
                    sceneViewAnimations.description = "The \"Always Refresh\" option is disabled in the scene view. Water surface animations will appear to be jumpy";

                    sceneViewAnimations.actionName = "Enable";
                    sceneViewAnimations.action = () =>
                    {
                        SceneView.lastActiveSceneView.sceneViewState.alwaysRefresh = true;
                    };
                    AddItem(sceneViewAnimations);
                }
            }

            /*
            //Test formatting
            SetupItem error = new SetupItem("Error");
            {
                error.state = MessageType.Error;
                error.description = "An error has occured";

                error.actionName = "Fix";
                error.action = () =>
                {

                };
                AddItem(error);
            }

            SetupItem warnings = new SetupItem("Warning");
            {
                warnings.state = MessageType.Warning;
                warnings.description = "Lorem Ipsum is simply dummy text of the printing and typesetting industry. Lorem Ipsum has been the industry's standard dummy text ever since the 1500s, when an unknown printer took a galley of type and scrambled it to make a type specimen book.";

                warnings.actionName = "Enable";
                warnings.action = () =>
                {

                };
                AddItem(warnings);
            }
            */

            //Sort to display errors first, then warnings.
            SetupItems = SetupItems.OrderBy(o=> (o.state == MessageType.Info || o.state
                 == MessageType.None)).ToList();
        }

        public static void OpenWindowIfNeeded()
        {
            if (m_errorCount > 0 && Application.isBatchMode == false && BuildPipeline.isBuildingPlayer == false)
            {
                HelpWindow.ShowWindow(true);
            }
        }

        public static void InstallPackage(string id)
        {
            SearchRequest request = Client.Search(id, false);

            while (request.Status == StatusCode.InProgress)
            {
                /* Waiting... */
            }

            if (request.IsCompleted)
            {
                if (request.Result == null)
                {
                    Debug.LogError($"Searching for package {id} failed");
                    return;
                }

                PackageInfo packageInfo = request.Result[0];
                string packageName = packageInfo.name;
                
                //Installed in project?
                bool installed = packageInfo.resolvedPath != string.Empty;

                if (installed)
                {
                    Debug.Log($"{packageName} is already installed");
                    return;
                }
                
                Debug.Log($"Installation of package \"{packageName}\" will start in a moment...");
                AddRequest addRequest = Client.Add(packageInfo.name + "@" + packageInfo.versions.latestCompatible);
                
            }
        }
    }
}