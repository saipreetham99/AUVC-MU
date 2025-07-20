// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

#if SPLINES
#define DEPENDENCIES_PRESENT
#endif

using System;
using System.Collections.Generic;
using UnityEngine;
using Object = UnityEngine.Object;
using Random = System.Random;
#if SPLINES
using UnityEngine.Splines;
using Unity.Mathematics;
#endif
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace StylizedWater3.DynamicEffects
{
    [ExecuteAlways]
    public class ShorelineWaveSpawner : MonoBehaviour
    {
        #if DEPENDENCIES_PRESENT
        public SplineContainer splineContainer;
        bool m_SplineDirty = false;
        
        [Space]
        
        public GameObject waveEffect;
        
        [Space]
        
        [Min(10f)]
        public float waveDistanceBetween = 25f;
        public bool flipDirection = false;
        public float waveDistanceFromShore = 40f;
        public float randomOffset = 15f;
        public float randomRotation = 5f;
        [SerializeField]
        public int seed = 0;

        [SerializeField] [HideInInspector]
        private GameObject[] waveInstances = Array.Empty<GameObject>();

        [SerializeField] [HideInInspector]
        private GameObject m_InstancesRoot;
        public bool hideInstances = true;

        [Space]
        
        [Tooltip("Assign an audio source to snap to the spline curve and follow the camera." +
                 "\n\nNote: The audio source will be moving, so set the Doppler Level to 0!")]
        public AudioSource audioSource;
        private AudioListener audioListener;
        private float m_audioDistanceNormalized;

        /// <summary>
        /// Normalized distance of the current audio listener to the shoreline spline. Intended to be used for parameterized audio such as FMOD
        /// </summary>
        public float ListenerDistanceNormalized => m_audioDistanceNormalized;
        
        public void Randomize()
        {
            seed = (int)UnityEngine.Random.Range(0, 9999);
            m_SplineDirty = true;
        }

        private void OnEnable()
        {
            Spline.Changed += OnSplineChanged;
        }

        private void OnDisable()
        {
            Spline.Changed -= OnSplineChanged;
        }
        
        
        void OnSplineChanged(Spline spline, int knotIndex, SplineModification modificationType)
        {
            if (splineContainer != null && splineContainer.Spline == spline)
            {
                m_SplineDirty = true;
            }
        }

        private void OnValidate()
        {
            m_SplineDirty = true;
        }

        private void Update()
        {
            if (m_SplineDirty)
            {
                Respawn();
            }

            if (audioSource)
            {
                UpdateAudio();
            }
        }

        private void UpdateAudio()
        {
            if (!audioListener) audioListener = FindFirstObjectByType<AudioListener>();
            if (!audioListener) return;

            Vector3 earPosition = audioListener.transform.position;

            #if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                if (SceneView.lastActiveSceneView) earPosition = SceneView.lastActiveSceneView.camera.transform.position;
            }
            #endif

            earPosition = splineContainer.transform.InverseTransformPoint(earPosition);
            SplineUtility.GetNearestPoint(splineContainer.Spline, earPosition, out var nearestPoint, out _, SplineUtility.PickResolutionMin, 1);

            m_audioDistanceNormalized = math.distance(earPosition, nearestPoint);
            
            audioSource.dopplerLevel = 0;
            audioSource.transform.position = splineContainer.transform.TransformPoint(nearestPoint);
        }

        private void Reset()
        {
            splineContainer = GetComponent<SplineContainer>();
        }

        void Respawn()
        {
            m_SplineDirty = false;
            
            if (waveInstances.Length > 0)
            {
                for (int i = 0; i < waveInstances.Length; i++)
                {
                    if(waveInstances[i]) DestroyImmediate(waveInstances[i]);
                }
                waveInstances = Array.Empty<GameObject>();
            }

            if (waveEffect && splineContainer)
            {
                float splineLength = splineContainer.Spline.CalculateLength(splineContainer.transform.localToWorldMatrix);
                int spawnPoints = Mathf.CeilToInt(splineLength/ waveDistanceBetween);
                
                List<GameObject> waves = new List<GameObject>();

                float3 position = Vector3.zero;
                float3 prevPos = Vector3.zero;

                float3 tangent = Vector3.forward;
                float3 up = Vector3.up;
                
                for (int i = 0; i <= spawnPoints; i++)
                {
                    float t = (float)i / spawnPoints;

                    splineContainer.Spline.Evaluate(t, out position, out tangent, out up);
                    float3 right = math.normalizesafe(math.cross(tangent, up));
                    if (flipDirection) right = -right;
                    
                    UnityEngine.Random.InitState(seed + i);
                    
                    position -= right * (waveDistanceFromShore + UnityEngine.Random.Range(0f, randomOffset));
                    position = splineContainer.transform.TransformPoint(position);
                    right = splineContainer.transform.TransformDirection(right);
                    
                    if (i == 0) prevPos = position;

                    if (math.distance(position, prevPos) > waveDistanceBetween)
                    {
                        GameObject wave = SpawnObject(waveEffect, this.transform);
                        
                        wave.gameObject.hideFlags = hideInstances ? HideFlags.HideInHierarchy : HideFlags.None;
                        
                        quaternion rotation = quaternion.LookRotationSafe(
                            Quaternion.AngleAxis(UnityEngine.Random.Range(-randomRotation, randomRotation), up) * right, up);

                        wave.transform.SetPositionAndRotation(position, rotation);
                        
                        waves.Add(wave);
                    }
                }

                waveInstances = waves.ToArray();
            }
        }

        private GameObject SpawnObject(GameObject source, Transform parent)
        {
            bool sourceIsPrefab = false;

            #if UNITY_EDITOR
            sourceIsPrefab = PrefabUtility.GetPrefabAssetType(source) != PrefabAssetType.NotAPrefab;

            if (sourceIsPrefab)
            {
                if (PrefabUtility.GetPrefabAssetType(source) != PrefabAssetType.Variant)
                {
                    source = PrefabUtility.GetCorrespondingObjectFromOriginalSource(source);
                }
            }
            #endif
            
            GameObject newObj = null;
            
            if (sourceIsPrefab)
            {
                #if UNITY_EDITOR
                newObj = (GameObject)PrefabUtility.InstantiatePrefab(source, parent);
                #endif
            }
            else
            {
                newObj = (GameObject)GameObject.Instantiate(source, parent);
                newObj.name = newObj.name.Replace("(Clone)", string.Empty);
            }

            return newObj;
        }

        private void OnDrawGizmosSelected()
        {
            if (waveInstances != null)
            {
                for (int i = 0; i < waveInstances.Length; i++)
                {
                    if(waveInstances[i]) Gizmos.DrawSphere(waveInstances[i].transform.position, 1f);
                }
            }

            if (audioSource)
            {
                Gizmos.DrawSphere(audioSource.transform.position, 1f);
                Gizmos.DrawWireSphere(audioSource.transform.position, audioSource.maxDistance);
            }
        }
        #endif
    }
}