// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.
#if (ENABLE_INPUT_SYSTEM && INPUT_SYSTEM_INSTALLED)
#define USE_INPUT_SYSTEM
#endif


using System;
using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif
#if USE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace StylizedWater3.Demo
{
    [ExecuteAlways]
    public class DemoLightingController : MonoBehaviour
    {
        [Serializable]
        public class Preset
        {
            public string name;
            public Material skybox;

            [Header("Direct light")]
    
            [Range(0f, 90f)]
            public float sunAngle = 45;
            [Range(0f, 360f)]
            public float sunRotation = 0f;

            public float intensity = 1f;
            public Color tint = Color.white;

            [Header("Indirect light")]
            public Color ambientColor = Color.gray;
            
            [Header("Fog")]
            public Color fogColor = Color.white;
            [Range(0.0001f, 0.01f)]
            public float fogDensity = 0.002f;
        }

        public static bool ShowGUI = true;

        [Min(0)]
        public int activeIndex = 0;
        public Preset[] presets = Array.Empty<Preset>();

        public ReflectionProbe reflectionProbe;
        
        [NonSerialized]
        private Material m_skybox;
        private Light sun;
        
        [SerializeField]
        private bool realtimeReflectionProbesDisabled;
        
        #if USE_INPUT_SYSTEM
        private InputAction[] numberKeyActions;
        #endif  
        
        private void OnEnable()
        {
            realtimeReflectionProbesDisabled = QualitySettings.realtimeReflectionProbes;

            if (!realtimeReflectionProbesDisabled)
            {
                //Debug.LogWarning("Realtime Reflection Probes are disabled in your Quality Settings, this is by default in new Unity projects. To ensure the water looks correct, it has been enabled temporarily.");
                //QualitySettings.realtimeReflectionProbes = true;
            }

            ApplyPreset(activeIndex);
            
            #if UNITY_EDITOR
            UnityEditor.SceneView.duringSceneGui += OnSceneGUI;
            #endif

            SetupInput();
        }
        
        private void SetupInput()
        {
            #if USE_INPUT_SYSTEM
            numberKeyActions = new InputAction[9];

            for (int i = 0; i < 9; i++)
            {
                int presetIndex = i;
                numberKeyActions[i] = new InputAction($"Preset{presetIndex + 1}", binding: $"<Keyboard>/{presetIndex + 1}");
                numberKeyActions[i].performed += ctx => OnNumberKeyPressed(presetIndex);
                numberKeyActions[i].Enable();
            }
            #endif
        }

        private void OnDisable()
        {
            RenderSettings.defaultReflectionMode = DefaultReflectionMode.Skybox;
            
            #if UNITY_EDITOR
            UnityEditor.SceneView.duringSceneGui -= OnSceneGUI;
            #endif
            
            //Do not meddle with project settings, restore changes
            if (realtimeReflectionProbesDisabled == false && QualitySettings.realtimeReflectionProbes == true) QualitySettings.realtimeReflectionProbes = false;
            
            #if USE_INPUT_SYSTEM
            if (numberKeyActions != null)
            {
                foreach (var action in numberKeyActions)
                {
                    action.Disable();
                    action.Dispose();
                }
            }
            #endif
        }
        
        private readonly int SkyboxTexID = Shader.PropertyToID("_Tex");

        public void ApplyPreset(int index = -1)
        {
            if (index < 0) index = activeIndex;
            
            if (this.gameObject.activeInHierarchy == false) return;
            if (index > presets.Length) return;

            activeIndex = index;
            
            Preset preset = presets[index];

            Light[] lights = FindObjectsByType<Light>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            for (int i = 0; i < lights.Length; i++)
            {
                if (lights[i].type == LightType.Directional) sun = lights[i];
            }

            if (m_skybox == null || preset.skybox.GetTexture(SkyboxTexID) != RenderSettings.skybox.GetTexture(SkyboxTexID))
            {
                CreateSkyboxMat(preset.skybox);
            }
            
            sun.intensity = preset.intensity;
            sun.color = preset.tint;
            
            m_skybox.CopyPropertiesFromMaterial(preset.skybox);
            m_skybox.SetTexture(SkyboxTexID, preset.skybox.GetTexture(SkyboxTexID));
            
            sun.transform.eulerAngles = new Vector3(preset.sunAngle, preset.sunRotation, 0f);
            m_skybox.SetFloat("_Rotation", -sun.transform.eulerAngles.y);
            
            RenderSettings.skybox = m_skybox;
            RenderSettings.fogColor = preset.fogColor;
            RenderSettings.fogDensity = preset.fogDensity;
            
            RenderSettings.ambientLight = preset.ambientColor;

            if (reflectionProbe)
            {
                reflectionProbe.RenderProbe();
                //RenderSettings.defaultReflectionMode = DefaultReflectionMode.Custom;
                //RenderSettings.customReflectionTexture = reflectionProbe.texture;
            }
            else
            {
                //RenderSettings.defaultReflectionMode = DefaultReflectionMode.Skybox;
            }
        }
        
        private void CreateSkyboxMat(Material source)
        {
            m_skybox = new Material(source);
            m_skybox.name = "Temp skybox";
        }
        
        private void OnNumberKeyPressed(int index)
        {
            if (index < presets.Length)
            {
                ApplyPreset(index);
            }
        }
        
        #if UNITY_EDITOR
        private void OnSceneGUI(SceneView sceneView)
        {
            Handles.BeginGUI();
            OnGUI();
            Handles.EndGUI();
        }
        #endif

        private void OnGUI()
        {
            if (!ShowGUI) return;
            
            using (new GUILayout.HorizontalScope(GUILayout.Width(300f)))
            {
                GUILayout.Label("  Lighting Presets:", GUI.skin.label);

                for (int i = 0; i < presets.Length; i++)
                {
                    GUI.enabled = (activeIndex != i);
                    if (GUILayout.Button(presets[i].name))
                    {
                        ApplyPreset(i);
                        
                        #if UNITY_EDITOR
                        UnityEditor.EditorUtility.SetDirty(this);
                        #endif
                    }
                }
                
                GUI.enabled = true;
            }
        }
    }
    
    #if UNITY_EDITOR
    [CustomEditor(typeof(DemoLightingController))]
    public class DemoLightingControllerEditor : Editor
    {
        private DemoLightingController component;
        private SerializedProperty presets;

        private SerializedProperty reflectionProbe;
        
        private string proSkinPrefix => EditorGUIUtility.isProSkin ? "d_" : "";
        
        private void OnEnable()
        {
            component = (DemoLightingController)target;
            presets = serializedObject.FindProperty("presets");
            reflectionProbe = serializedObject.FindProperty("reflectionProbe");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUI.BeginChangeCheck();

            EditorGUILayout.PropertyField(reflectionProbe);
            DemoLightingController.ShowGUI = EditorGUILayout.Toggle("Show GUI", DemoLightingController.ShowGUI);
            
            EditorGUILayout.Space();
            
            EditorGUILayout.LabelField("Presets", EditorStyles.boldLabel);
            
            for (int i = 0; i < presets.arraySize; i++)
            {
                if (GUILayout.Button("Set Active"))
                {
                    component.activeIndex = i;
                    component.ApplyPreset(i);
                    
                    EditorUtility.SetDirty(component);
                }

                using (new EditorGUI.DisabledGroupScope(component.activeIndex != i))
                {
                    EditorGUI.BeginChangeCheck();
                    
                    using (new EditorGUILayout.HorizontalScope(EditorStyles.helpBox))
                    {
                        using (new EditorGUILayout.VerticalScope())
                        {
                            GUILayout.Space(5f);

                            SerializedProperty param = presets.GetArrayElementAtIndex(i);
                            
                            EditorGUILayout.PropertyField(param);

                            GUILayout.Space(5f);
                        }

                        if (GUILayout.Button(new GUIContent("", EditorGUIUtility.IconContent(proSkinPrefix + "TreeEditor.Trash").image, "Remove parameter"), EditorStyles.miniButton, GUILayout.Width(30f))) presets.DeleteArrayElementAtIndex(i);

                    }
                    
                    if (EditorGUI.EndChangeCheck())
                    {
                        if (component.activeIndex == i)
                        {
                            component.ApplyPreset(i);
                        }
                    }
                }
                
                GUILayout.Space(3f);
            }

            using (new EditorGUILayout.HorizontalScope())
            {
                GUILayout.FlexibleSpace();

                if (GUILayout.Button(new GUIContent(" Add", EditorGUIUtility.IconContent(proSkinPrefix + "Toolbar Plus").image, "Insert new parameter"), EditorStyles.miniButton, GUILayout.Width(60f)))
                {
                    presets.InsertArrayElementAtIndex(presets.arraySize);
                }
            }

            if (EditorGUI.EndChangeCheck())
            {
                serializedObject.ApplyModifiedProperties();
            }
        }
    }
    #endif
    
}