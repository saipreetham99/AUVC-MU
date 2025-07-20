// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using UnityEditor;
using UnityEngine;
#if URP
using UnityEngine.Rendering.Universal;
#endif

namespace StylizedWater3.DynamicEffects
{
    [CustomEditor(typeof(DynamicEffect))]
    [CanEditMultipleObjects]
    public class DynamicEffectInspector : Editor
    {
        private SerializedProperty renderer;
        private SerializedProperty sortingLayer;

        private SerializedProperty templateMaterial;
        
        private SerializedProperty heightScale;
        private SerializedProperty foamAmount;
        private SerializedProperty normalStrength;

        private SerializedProperty scaleHeightByTransform;
        private SerializedProperty scaleNormalByHeight;

        #if URP
        private bool renderFeaturePresent;
        private bool renderFeatureEnabled;
        private StylizedWaterRenderFeature renderFeature;
        private Editor renderFeatureEditor;
        #endif
        
        private void OnEnable()
        {
            renderer = serializedObject.FindProperty("renderer");
            sortingLayer = serializedObject.FindProperty("sortingLayer");
            
            templateMaterial = serializedObject.FindProperty("templateMaterial");
            
            heightScale = serializedObject.FindProperty("heightScale");
            foamAmount = serializedObject.FindProperty("foamAmount");
            normalStrength = serializedObject.FindProperty("normalStrength");
            
            scaleHeightByTransform = serializedObject.FindProperty("scaleHeightByTransform");
            scaleNormalByHeight = serializedObject.FindProperty("scaleNormalByHeight");
            
            #if URP
            renderFeaturePresent = PipelineUtilities.RenderFeatureAdded<StylizedWaterRenderFeature>();
            renderFeatureEnabled = PipelineUtilities.IsRenderFeatureEnabled<StylizedWaterRenderFeature>();
            renderFeature = PipelineUtilities.GetRenderFeature<StylizedWaterRenderFeature>() as StylizedWaterRenderFeature;
            #endif

            DynamicEffect component = (DynamicEffect)target;
            if (component.renderer && component.templateMaterial == null)
            {
                component.templateMaterial = component.renderer.sharedMaterial;
                EditorUtility.SetDirty(target);
            }
        }
        
        private void DrawNotifications()
        {
            #if URP
            UI.DrawNotification( !AssetInfo.MeetsMinimumVersion(DynamicEffects.extension.minBaseVersion), "Version mismatch, requires Stylized Water 3 v" + DynamicEffects.extension.minBaseVersion +".\n\nUpdate to avoid any issues or resolve (shader) errors", "Update", () => AssetInfo.OpenInPackageManager(), MessageType.Error);
            
            UI.DrawNotification(UniversalRenderPipeline.asset == null, "The Universal Render Pipeline is not active", MessageType.Error);
            
            using (new EditorGUI.DisabledGroupScope(Application.isPlaying))
            {
                UI.DrawNotification(!renderFeaturePresent, "The Dynamic Effects render feature hasn't be added to the default renderer", "Add", () =>
                {
                    PipelineUtilities.AddRenderFeature<StylizedWaterRenderFeature>();
                    renderFeaturePresent = true;
                    renderFeature = PipelineUtilities.GetRenderFeature<StylizedWaterRenderFeature>() as StylizedWaterRenderFeature;
                }, MessageType.Error);
            }
            if(Application.isPlaying && !renderFeaturePresent) EditorGUILayout.HelpBox("Exit play mode to perform this action", MessageType.Warning);
            
            UI.DrawNotification(renderFeaturePresent && !renderFeatureEnabled, "The Dynamic Effects render feature is disabled", "Enable", () => 
            { 
                PipelineUtilities.ToggleRenderFeature<StylizedWaterRenderFeature>(true);
                renderFeature.dynamicEffectsSettings.enabled = true;
                renderFeatureEnabled = true; 
            }, MessageType.Warning);
            #else
            UI.DrawNotification("The Universal Render Pipeline package is not installed!", MessageType.Error);
            #endif
        }
        
        public override void OnInspectorGUI()
        {
            EditorGUILayout.LabelField($"{AssetInfo.ASSET_NAME }: Dynamic Effects v{DynamicEffects.extension.version}", EditorStyles.centeredGreyMiniLabel);
            EditorGUILayout.Space();
            
            DrawNotifications();

            serializedObject.Update();
            
            EditorGUI.BeginChangeCheck();

            EditorGUILayout.PropertyField(renderer);

            if (renderer.objectReferenceValue)
            {
                EditorGUILayout.BeginHorizontal();
                {
                    EditorGUILayout.PrefixLabel(new GUIContent(sortingLayer.displayName, sortingLayer.tooltip));

                    if (GUILayout.Button("-", EditorStyles.miniButtonLeft, GUILayout.Width(20)))
                    {
                        sortingLayer.intValue--;
                    }

                    EditorGUILayout.PropertyField(sortingLayer, GUIContent.none, GUILayout.MaxWidth(32));

                    if (GUILayout.Button("+", EditorStyles.miniButtonRight, GUILayout.Width(20)))
                    {
                        sortingLayer.intValue++;
                    }
                }
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.Space();

                bool hasMaterial = templateMaterial.objectReferenceValue;
                EditorGUILayout.LabelField("Material", EditorStyles.boldLabel);
                using (new EditorGUILayout.HorizontalScope())
                {
                    EditorGUILayout.PropertyField(templateMaterial);
                    
                    
                    EditorGUI.BeginDisabledGroup(hasMaterial == false);
                    if (GUILayout.Button("Edit", EditorStyles.miniButton, GUILayout.Width(50f)))
                    {
                        StylizedWaterEditor.PopUpMaterialEditor.Create(templateMaterial.objectReferenceValue);

                        /*
                        MaterialEditor materialEditor = ScriptableObject.CreateInstance<MaterialEditor>();
                        materialEditor.target = templateMaterial.objectReferenceValue as Material;
                        materialEditor.
                        */
                        //Selection.activeObject = templateMaterial.objectReferenceValue;
                    }
                    EditorGUI.EndDisabledGroup();
                }
                
                UI.DrawNotification(hasMaterial == false, "A material using the \"Dynamic Effects\" shader must be assigned", MessageType.Error);

                if (hasMaterial)
                {
                    EditorGUILayout.Space();

                    EditorGUILayout.PropertyField(heightScale);
                    EditorGUI.indentLevel++;
                    EditorGUILayout.PropertyField(scaleHeightByTransform, new GUIContent("Scale with Transform", scaleHeightByTransform.tooltip));
                    EditorGUI.indentLevel--;
                    EditorGUILayout.PropertyField(foamAmount);
                    EditorGUILayout.PropertyField(normalStrength);
                    EditorGUI.indentLevel++;
                    EditorGUILayout.PropertyField(scaleNormalByHeight, new GUIContent("Scale by height", scaleNormalByHeight.tooltip));
                    EditorGUI.indentLevel--;
                }
                
                EditorGUILayout.Space();

            }
            else
            {
                UI.DrawNotification("A renderer must be assigned", MessageType.Error);
            }

            if (EditorGUI.EndChangeCheck())
            {
                serializedObject.ApplyModifiedProperties();
                
                DynamicEffect script = (DynamicEffect)target;
                script.UpdateMaterial();
            }

            UI.DrawFooter();
        }
    }
}