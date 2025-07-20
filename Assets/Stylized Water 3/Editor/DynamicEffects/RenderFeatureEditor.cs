// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

#if URP
using System;
using UnityEditor;
using UnityEngine;
using StylizedWater3.DynamicEffects;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace StylizedWater3
{
    public partial class RenderFeatureEditor : Editor
    {
        private SerializedProperty dynamicEffectsSettings;
        
        private SerializedProperty enabled;
        
        private SerializedProperty renderRange;
        private SerializedProperty fadePercentage;
        
        private SerializedProperty texelsPerUnit;
        private SerializedProperty maxResolution;

        private SerializedProperty enableDisplacement;
        private SerializedProperty enableNormals;
        private SerializedProperty halfResolutionNormals;
        private SerializedProperty normalMipmaps;

        private SerializedProperty ignoreSceneView;
        
        private SerializedProperty heightProcessingShader;
        
        partial void DynamicEffectsOnEnable()
        {
            dynamicEffectsSettings = serializedObject.FindProperty("dynamicEffectsSettings");
            
            enabled = dynamicEffectsSettings.FindPropertyRelative("enabled");
            
            renderRange = dynamicEffectsSettings.FindPropertyRelative("renderRange");
            fadePercentage = dynamicEffectsSettings.FindPropertyRelative("fadePercentage");
            
            texelsPerUnit = dynamicEffectsSettings.FindPropertyRelative("texelsPerUnit");
            maxResolution = dynamicEffectsSettings.FindPropertyRelative("maxResolution");
            
            enableDisplacement = dynamicEffectsSettings.FindPropertyRelative("enableDisplacement");
            enableNormals = dynamicEffectsSettings.FindPropertyRelative("enableNormals");
            halfResolutionNormals = dynamicEffectsSettings.FindPropertyRelative("halfResolutionNormals");
            normalMipmaps = dynamicEffectsSettings.FindPropertyRelative("normalMipmaps");
            
            ignoreSceneView = dynamicEffectsSettings.FindPropertyRelative("ignoreSceneView");

            heightProcessingShader = serializedObject.FindProperty("heightProcessingShader");

            if (target.name == "NewWaterDynamicEffectsRenderFeature") target.name = "Stylized Water 3: Dynamic Effects";
        }
        
        partial void DynamicEffectsOnInspectorGUI()
        {
            EditorGUILayout.LabelField("Dynamic Effects", EditorStyles.boldLabel);
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.LabelField($"Version {DynamicEffects.DynamicEffects.extension.version}", EditorStyles.miniLabel);

                if (GUILayout.Button(new GUIContent(" Documentation", EditorGUIUtility.FindTexture("_Help"))))
                {
                    Application.OpenURL("https://staggart.xyz/unity/stylized-water-3/sw3-dynamic-effects-docs/");
                }
            }
            EditorGUILayout.Space();

            serializedObject.Update();
            EditorGUI.BeginChangeCheck();

            EditorGUILayout.PropertyField(enabled);
            
            EditorGUILayout.HelpBox($"Active effects in scene: {DynamicEffect.InstanceCount} {(DynamicEffect.InstanceCount == 0 ? "(Rendering is skipped)" : "")}", MessageType.None);

            if (enabled.boolValue)
            {
                EditorGUILayout.PropertyField(renderRange);
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(fadePercentage, new GUIContent("Fade range (%)", fadePercentage.tooltip));
                EditorGUI.indentLevel--;

                EditorGUILayout.Space();

                EditorGUILayout.PropertyField(texelsPerUnit);
                EditorGUILayout.PropertyField(maxResolution);
                using (new EditorGUILayout.HorizontalScope())
                {
                    GUILayout.Space(EditorGUIUtility.labelWidth);

                    EditorGUILayout.HelpBox($"Current resolution: {PlanarProjection.CalculateResolution(renderRange.floatValue, texelsPerUnit.intValue, StylizedWaterRenderFeature.DynamicEffectsSettings.MIN_RESOLUTION, maxResolution.intValue)}px", MessageType.None);
                }

                EditorGUILayout.Space();

                EditorGUILayout.PropertyField(enableDisplacement);

                EditorGUILayout.PropertyField(enableNormals);
                using (new EditorGUI.DisabledGroupScope(enableNormals.boolValue == false))
                {
                    EditorGUI.indentLevel++;
                    EditorGUILayout.PropertyField(normalMipmaps);
                    if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Direct3D12)
                    {
                        EditorGUILayout.HelpBox("Mipmap rendering is broken in DirectX 12, so will internally be disabled", MessageType.Warning);
                    }
                    EditorGUILayout.PropertyField(halfResolutionNormals);
                    EditorGUI.indentLevel--;
                }

                EditorGUILayout.Space();

                EditorGUILayout.PropertyField(ignoreSceneView);
            }
            
            if (EditorGUI.EndChangeCheck())
            {
                serializedObject.ApplyModifiedProperties();
            }
            
            //base.OnInspectorGUI();

            if (heightProcessingShader.objectReferenceValue == null)
            {
                EditorGUILayout.HelpBox("Internal shader not referenced!", MessageType.Error);
                if (GUILayout.Button("Find & assign"))
                {
                    heightProcessingShader.objectReferenceValue = Shader.Find("Hidden/StylizedWater3/HeightProcessor");
                    serializedObject.ApplyModifiedProperties();
                }
            }
        }
    }
}
#endif