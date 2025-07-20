using System;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace StylizedWater3
{
    /// <summary>
    /// Emulates the particle system "Rate over Distance" emission behaviour, but with accurate support for RigidBody's
    /// </summary>
    [ExecuteInEditMode]
    [AddComponentMenu("Effects/Particle Trail Emitter")]
    public class ParticleTrailEmitter : MonoBehaviour
    {
#pragma warning disable 108,114 //New keyword
        [Tooltip("The Emission module on this particle system should have its Rate Over Distance value set 0.")]
        public ParticleSystem particleSystem;
        private ParticleSystem.EmissionModule emissionModule;
#pragma warning restore 108,114

        [Tooltip("If this particle system is parented under a RigidBody, then assign it here for correct positional tracking")]
        public Rigidbody rigidBody;

        [Space]

        public float spawnRatePerUnit = 1f;

        private float distanceAccumulation = 0f;
        private Vector3 previousPosition;
        
        void Reset()
        {
            particleSystem = GetComponent<ParticleSystem>();

            if (particleSystem)
            {
                emissionModule = particleSystem.emission;

                if (emissionModule.rateOverDistance.constant > 0f)
                {
                    ParticleSystem.MinMaxCurve rateOverDistance = emissionModule.rateOverDistance;
                    rateOverDistance.constant = 0f;
                    emissionModule.rateOverDistance = rateOverDistance;
                    
                    Debug.LogWarning($"The Rate Over Distance has been set to 0 on the particle system \"{particleSystem.name}\". This is because the Particle Trail Emitter component will be responsible for emission");
                }
            }
            rigidBody = GetComponentInParent<Rigidbody>();
        }

        private void Start()
        {
            previousPosition = this.transform.position;

            if (particleSystem)
            {
                ParticleSystem.MainModule main = particleSystem.main;
                main.playOnAwake = false;
            }
        }

        private float GetDistance()
        {
            float distanceThisFrame = 0;
            
            if(rigidBody)
            {
                //Distance = speed * deltaTime
                float movementSpeed = Mathf.Max(rigidBody.linearVelocity.magnitude, rigidBody.angularVelocity.magnitude);
                distanceThisFrame = movementSpeed * Time.deltaTime;
            }
            else
            {
                distanceThisFrame = Vector3.Distance(transform.position, previousPosition);
                previousPosition = this.transform.position;
            }

            return distanceThisFrame;
        }

        public void FixedUpdate()
        {
            if (!particleSystem) return;

            emissionModule = particleSystem.emission;
            if (emissionModule.enabled == false) return;
            
            distanceAccumulation += GetDistance();

            var particlesToEmit = Mathf.CeilToInt(distanceAccumulation * spawnRatePerUnit);

            if (particlesToEmit > 0)
            {
                particleSystem.Emit(particlesToEmit);
                distanceAccumulation -= particlesToEmit / spawnRatePerUnit;
            }
        }
    }
    
    #if UNITY_EDITOR
    [CustomEditor(typeof(ParticleTrailEmitter))]
    class ParticleTrailEmitterInspector : Editor
    {
        private ParticleTrailEmitter component;
        
        private void OnEnable()
        {
            component = (ParticleTrailEmitter)target;
        }
        
        private void OnSceneGUI()
        {
            component.FixedUpdate();
        }
    }
    #endif
}