// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Audio;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace StylizedWater3.UnderwaterRendering
{
	[CustomEditor(typeof(UnderwaterArea))]
	class UnderwaterAreaEditor : Editor
	{
		private UnderwaterArea component;
		
		private SerializedProperty waterMaterial;
		private SerializedProperty renderingOrder;
		private SerializedProperty boxCollider;
		
		private SerializedProperty waterLevelSource;
		private SerializedProperty waterLevel;
		private SerializedProperty waterLevelTransform;

		private SerializedProperty waterlineOffset;
		private SerializedProperty waterlineThickness;
		
		private SerializedProperty shadingSettings;
		private SerializedProperty particleEffects;
		private SerializedProperty environmentAudioMixer;
		private SerializedProperty lowPassCutoffFrequency;

		private bool colliderTooLow;
		private bool shaderMissing;
		private bool opaqueDownsampled;
		private bool refractionEnabled;
		private bool needsv320Upgrade;
		
		private StylizedWaterRenderFeature renderFeature;
		private void OnEnable()
		{
			component = (UnderwaterArea)target;
			
			waterMaterial = serializedObject.FindProperty("waterMaterial");
			renderingOrder = serializedObject.FindProperty("renderingOrder");
			boxCollider = serializedObject.FindProperty("boxCollider");
			
			waterLevelSource = serializedObject.FindProperty("waterLevelSource");
			waterLevel = serializedObject.FindProperty("waterLevel");
			waterLevelTransform = serializedObject.FindProperty("waterLevelTransform");
			
			waterlineOffset = serializedObject.FindProperty("waterlineOffset");
			waterlineThickness = serializedObject.FindProperty("waterlineThickness");
			
			shadingSettings = serializedObject.FindProperty("shadingSettings");
			particleEffects = serializedObject.FindProperty("particleEffects");
			environmentAudioMixer = serializedObject.FindProperty("environmentAudioMixer");
			lowPassCutoffFrequency = serializedObject.FindProperty("lowPassCutoffFrequency");

			ValidateCollider();

			shaderMissing = component.HasValidShader() == false;
			opaqueDownsampled = PipelineUtilities.IsOpaqueDownSampled();
			if(waterMaterial.objectReferenceValue) refractionEnabled = ((Material)waterMaterial.objectReferenceValue).IsKeywordEnabled(ShaderParams.Keywords.Refraction);

			renderFeature = StylizedWaterRenderFeature.GetDefault();

			needsv320Upgrade = HasHiddenOldHiddenObjects();
		}

		private void ValidateCollider()
		{
			if (boxCollider.objectReferenceValue != null && component.boxCollider)
			{
				colliderTooLow = component.GetColliderPlaneHeight() < component.CurrentWaterLevel;
				
				//Debug.Log($"Collider: {component.GetColliderPlaneHeight()}. Water level: {component.CurrentWaterLevel}. colliderTooLow: {colliderTooLow}");
			}
		}

		//Validation when upgrading to v3.2.0
		private bool HasHiddenOldHiddenObjects()
		{
			Transform[] childs = component.GetComponentsInChildren<Transform>();

			for (int i = 0; i < childs.Length; i++)
			{
				if (childs[i].gameObject.name == "Underwater Glass Surface") return true;
			}

			return false;
		}

		public override void OnInspectorGUI()
		{
			EditorGUILayout.LabelField("Version " + UnderwaterRendering.extension.version, EditorStyles.centeredGreyMiniLabel);

			UI.DrawNotification(opaqueDownsampled && refractionEnabled, "The Opaque texture is not rendering at full resolution. Water will appear blurry and outlines may appear on geometry", "Fix", () =>
			{
				PipelineUtilities.DisableOpaqueDownsampling();

				opaqueDownsampled = false;
			}, MessageType.Warning);
			
			if (shaderMissing)
			{
				UI.DrawNotification(true, "Underwater rendering shaders are missing. It has possibly: not been assigned to the component, was deleted, or failed to compile", "Try finding", () =>
				{
					component.FindShaders();
					component.Reset();
					EditorUtility.SetDirty(component);

					shaderMissing = false;
				}, MessageType.Error);
				return;
			}
			
			UI.DrawNotification(renderFeature == null, "The Stylized Water render feature hasn't been set up on the current renderer. Underwater shading will not work correctly.", MessageType.Error);
			
			UI.DrawNotification(needsv320Upgrade, "This component needs to be upgraded to v3.2.0+", "Do it", () =>
			{
				Transform[] childs = component.GetComponentsInChildren<Transform>();
				
				for (int i = 0; i < childs.Length; i++)
				{
					if (childs[i].gameObject.name == "Underwater Glass Surface") CoreUtils.Destroy(childs[i].gameObject);
				}
				
				EditorUtility.SetDirty(component);
				
				needsv320Upgrade = false;
			}, MessageType.Info);
			
			Camera currentCamera = Application.isPlaying ? Camera.main : SceneView.lastActiveSceneView?.camera;
			
			if (currentCamera)
			{
				this.Repaint();
				
				if (component.CameraIntersects(currentCamera))
				{

					float submersionPercentage = component.CameraSubmersionAmount(currentCamera) * 100f;
					
					EditorGUILayout.HelpBox($"Rendering for: {currentCamera.name} ({submersionPercentage}% submerged)", MessageType.None);
				}
				else
				{
					EditorGUILayout.HelpBox($"No camera currently inside the volume", MessageType.None);
				}
			}
			
			EditorGUILayout.Separator();
			
			serializedObject.Update();
			EditorGUI.BeginChangeCheck();

			using (new EditorGUILayout.HorizontalScope())
			{
				EditorGUILayout.PropertyField(waterMaterial);

				EditorGUI.BeginDisabledGroup(waterMaterial.objectReferenceValue == null);
				if (GUILayout.Button("Edit", EditorStyles.miniButton, GUILayout.Width(50f)))
				{
					Selection.activeObject = waterMaterial.objectReferenceValue;
				}
				EditorGUI.EndDisabledGroup();
			}

			if (waterMaterial.objectReferenceValue)
			{
				Material material = waterMaterial.objectReferenceValue as Material;
				UI.DrawNotification(material.GetInt("_Cull") != (int)CullMode.Off, "The water material is not double-sided", "Make it so", () =>{
					StylizedWaterEditor.DisableCullingForMaterial(material);
				}, MessageType.Error);

				if (renderFeature && renderFeature.underwaterRenderingSettings.renderMethod == StylizedWaterRenderFeature.UnderwaterRenderingSettings.RenderMethod.SceneRenderer)
				{
					EditorGUILayout.PropertyField(renderingOrder);
					EditorGUILayout.HelpBox($"{material.renderQueue} {(renderingOrder.intValue >= 0 ? "+ " : "")}{renderingOrder.intValue}", MessageType.None, false);
				}

				EditorGUILayout.Separator();
				
				EditorGUILayout.LabelField("Trigger volume", EditorStyles.boldLabel);
				
				EditorGUI.BeginChangeCheck();
				EditorGUILayout.PropertyField(boxCollider);
				if (EditorGUI.EndChangeCheck()) ValidateCollider();
				if (colliderTooLow)
				{
					
					UI.DrawNotification(true, $"The height of the volume is too low ({component.GetColliderPlaneHeight()}). It needs to extend above the water level height ({component.CurrentWaterLevel}).", "Fix", () =>
					{
						BoxCollider collider = boxCollider.objectReferenceValue as BoxCollider;

						float delta = Mathf.Abs(component.GetColliderPlaneHeight() - component.CurrentWaterLevel);

						//Padding
						delta += 3f;
						
						Vector3 center = collider.center;
						center.y += delta * 0.5f;
						collider.center = center;
						
						Vector3 size = collider.size;
						size.y += delta * 0.5f;
						collider.size = size;
						
						EditorUtility.SetDirty(collider);
						colliderTooLow = false;
						
					}, MessageType.Error);
				}
				
				if (boxCollider.objectReferenceValue)
				{
					BoxCollider collider = boxCollider.objectReferenceValue as BoxCollider;
					UI.DrawNotification(collider.isTrigger == false, "The box collider must be configured as a trigger", "Make it so", () =>
					{
						collider.isTrigger = true;
						EditorUtility.SetDirty(collider);
					}, MessageType.Error);
				}
				
				EditorGUILayout.Separator();
				
				EditorGUILayout.LabelField("Water level height", EditorStyles.boldLabel);
				
				EditorGUI.BeginChangeCheck();
				
				using (new EditorGUILayout.HorizontalScope())
				{
					EditorGUILayout.PropertyField(waterLevelSource);
					component.visualizeWaterLevel = GUILayout.Toggle(component.visualizeWaterLevel , new GUIContent("  Visualize", EditorGUIUtility.IconContent((component.visualizeWaterLevel ? "animationvisibilitytoggleon" : "animationvisibilitytoggleoff")).image), "Button");
				}
				
				EditorGUI.indentLevel++;
				if (waterLevelSource.intValue == (int)UnderwaterArea.WaterLevelSource.Transform)
				{
					EditorGUILayout.PropertyField(waterLevelTransform);
					
				}
				else if (waterLevelSource.intValue == (int)UnderwaterArea.WaterLevelSource.FixedValue)
				{
					EditorGUILayout.PropertyField(waterLevel);
				}

				if (waterLevelSource.intValue != (int)UnderwaterArea.WaterLevelSource.FixedValue)
				{
					EditorGUILayout.HelpBox($"Y={component.CurrentWaterLevel}", MessageType.None, false);
				}
				
				if (waterLevelSource.intValue == (int)UnderwaterArea.WaterLevelSource.Ocean && !OceanFollowBehaviour.Instance)
				{
					EditorGUILayout.HelpBox($"There is no ocean currently set up in the scene...", MessageType.Error, false);
				}
				
				if (EditorGUI.EndChangeCheck()) ValidateCollider();
				
				EditorGUI.indentLevel--;
				
				EditorGUILayout.Separator();
				
				EditorGUILayout.LabelField("Waterline rendering", EditorStyles.boldLabel);
				EditorGUILayout.PropertyField(waterlineOffset);
				EditorGUILayout.PropertyField(waterlineThickness);
				
				EditorGUILayout.Separator();
				
				EditorGUILayout.LabelField("Rendering", EditorStyles.boldLabel);
				EditorGUILayout.PropertyField(shadingSettings);
				EditorGUILayout.PropertyField(particleEffects);
				
				#if SWS_DEV
				EditorGUILayout.Separator();
				
				EditorGUILayout.LabelField("Audio", EditorStyles.boldLabel);
				EditorGUILayout.PropertyField(environmentAudioMixer);
				if (environmentAudioMixer.objectReferenceValue)
				{
					bool hasParam = ((AudioMixer)environmentAudioMixer.objectReferenceValue).GetFloat(UnderwaterArea.AUDIO_MIXER_LOWPASS_PARAM, out _);
					
					UI.DrawNotification(hasParam == false, $"The Audio Mixer does not have the \"{UnderwaterArea.AUDIO_MIXER_LOWPASS_PARAM}\" parameter exposed." +
					                                       $"\n\n" +
					                                       $"It requires further set up, check the documentation for instructions...");
					
				}
				EditorGUILayout.PropertyField(lowPassCutoffFrequency);
				#endif
			}
			else
			{
				EditorGUILayout.HelpBox("Assign the water material used by the water surface", MessageType.Info);
			}

			if (EditorGUI.EndChangeCheck())
			{
				ValidateCollider();
				
				serializedObject.ApplyModifiedProperties();
			}

			UI.DrawFooter();
		}

		[MenuItem("GameObject/3D Object/Water/Underwater Area (+VFX)")]
		public static void CreateUnderwaterAreaVFX()
		{
			CreateAreaObject(true);
		}
		
		[MenuItem("GameObject/3D Object/Water/Underwater Area")]
		public static void CreateUnderwaterArea()
		{
			CreateAreaObject(false);
		}

		private const string SUNSHAFT_PREFAB_GUID = "403a9af4ecfc8fa4db7cf2680a4bb3cd";
		private const string PLANKTON_PREFAB_GUID = "1547e00092a8c1847b40d67f29c0c299";
		private const string BUBBLES_PREFAB_GUID = "07d6ca6898263394c9e08e1b92a333ad";
		
		public static UnderwaterArea CreateAreaObject(bool vfx)
		{
			GameObject gameObject = new GameObject("Underwater Area");
			
			Undo.RegisterCreatedObjectUndo(gameObject, "Created Underwater Area");

			gameObject.layer = LayerMask.NameToLayer("Water");
			
			if (Selection.activeGameObject) gameObject.transform.parent = Selection.activeGameObject.transform;
            
			Selection.activeObject = gameObject;

			//Position in view
			if (SceneView.lastActiveSceneView)
			{
				Ray ray = new Ray(SceneView.lastActiveSceneView.camera.transform.position, SceneView.lastActiveSceneView.camera.transform.forward);
				
				Vector3 position = ray.origin + (ray.direction * 15f);

				if (Physics.Raycast(ray.origin, ray.direction, out var hit, 1000, -1, QueryTriggerInteraction.Ignore))
				{
					position = hit.point;
				}
				
				gameObject.transform.position = position;
			}
			
			UnderwaterArea area = gameObject.AddComponent<UnderwaterArea>();
            area.Reset();

            if (vfx)
            {
	            void AddVFX(string guid, float minDepth, float maxDepth, bool sunRotate)
	            {
		            string prefabPath = AssetDatabase.GUIDToAssetPath(guid);
		            if (!string.IsNullOrEmpty(prefabPath))
		            {
			            Object prefab = AssetDatabase.LoadAssetAtPath<Object>(prefabPath);
			            
			            GameObject instance = PrefabUtility.InstantiatePrefab(prefab) as GameObject;
			            instance.transform.parent = gameObject.transform;
			            
			            UnderwaterArea.ParticleEffect effect = new UnderwaterArea.ParticleEffect(minDepth, maxDepth, sunRotate);
			            
			            ParticleSystem particleSystem = instance.GetComponent<ParticleSystem>();
			            effect.particleSystem = particleSystem;
			            
			            area.particleEffects.Add(effect);
		            }
		            else
		            {
			            throw new Exception($"[{AssetInfo.ASSET_NAME}] Failed to load VFX prefab with GUID: {guid}. Was is deleted? Not imported? Meta files removed?");
		            }
	            }
	            
	            AddVFX(SUNSHAFT_PREFAB_GUID, 0f, 10, true);
	            AddVFX(PLANKTON_PREFAB_GUID, 0f, 100, false);
	            AddVFX(BUBBLES_PREFAB_GUID, 0f, 100, false);
	            
	            EditorUtility.SetDirty(area);
            }
            
			if(Application.isPlaying == false) EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());

			return area;
		}
	}
}