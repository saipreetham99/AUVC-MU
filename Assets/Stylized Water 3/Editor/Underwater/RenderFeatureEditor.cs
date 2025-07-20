// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using StylizedWater3.UnderwaterRendering;
using UnityEditor;
using UnityEngine;

namespace StylizedWater3
{
    public partial class RenderFeatureEditor : Editor
    {
        private SerializedProperty underwaterRenderingSettings;

        private SerializedProperty underwaterEnable;
        private SerializedProperty underwaterRenderMethod;
        private SerializedProperty underwaterIgnoreSceneView;
        
        partial void UnderwaterRenderingOnEnable()
        {
            underwaterRenderingSettings = serializedObject.FindProperty("underwaterRenderingSettings");
            
            underwaterEnable = underwaterRenderingSettings.FindPropertyRelative("enable");
            underwaterRenderMethod = underwaterRenderingSettings.FindPropertyRelative("renderMethod");
            underwaterIgnoreSceneView = underwaterRenderingSettings.FindPropertyRelative("ignoreSceneView");
        }
        
        partial void UnderwaterRenderingOnInspectorGUI()
        {
            EditorGUILayout.Space();
            
            EditorGUILayout.LabelField(StylizedWater3.UnderwaterRendering.UnderwaterRendering.extension.name, EditorStyles.boldLabel);
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.LabelField($"Version {StylizedWater3.UnderwaterRendering.UnderwaterRendering.extension.version}", EditorStyles.miniLabel);
            }

            serializedObject.Update();
            EditorGUI.BeginChangeCheck();
            
            EditorGUILayout.HelpBox($"Areas in scene: {UnderwaterArea.Instances.Count}", MessageType.None);

            EditorGUILayout.PropertyField(underwaterEnable);

            EditorGUILayout.Separator();
            
            if (underwaterEnable.boolValue)
            {
                EditorGUILayout.PropertyField(underwaterRenderMethod);

                if ((StylizedWaterRenderFeature.UnderwaterRenderingSettings.RenderMethod)underwaterRenderMethod.intValue == StylizedWaterRenderFeature.UnderwaterRenderingSettings.RenderMethod.RenderPass)
                {
                    UI.DrawNotification("Renders before all other Transparent materials.\n" +
                                         "\n• Supports multiple simultaneous cameras (eg. split-screen)" +
                                         "\n• Allows custom effects to execute before underwater shading" +
                                         "\n• Transparent materials appear unfogged, without altered shader"
                                         );
                }
                else
                {
                    UI.DrawNotification("Renders as a traditional Mesh Renderer in the scene.\n" +
                                        "\n• Allows setting the render queue" +
                                        "\n• Mixes with other transparent materials" +
                                        "\n• Best performance for mobile VR");
                }
                EditorGUILayout.PropertyField(underwaterIgnoreSceneView);
                
            }
            
            if (EditorGUI.EndChangeCheck())
            {
                serializedObject.ApplyModifiedProperties();
            }

            /*
            if (resources.objectReferenceValue == null)
            {
                EditorGUILayout.HelpBox("Internal shader resources object not referenced!", MessageType.Error);
                if (GUILayout.Button("Find & assign"))
                {
                    resources.objectReferenceValue = UnderwaterResources.Find();
                    serializedObject.ApplyModifiedProperties();
                }
            }
            */
        }
    }
}