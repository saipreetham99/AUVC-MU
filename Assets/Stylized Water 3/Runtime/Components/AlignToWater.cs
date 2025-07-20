// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using Unity.Mathematics;
using UnityEngine;

namespace StylizedWater3
{
    /// <summary>
    /// Samples the water height at 4 points around the Transform, and snaps the Y-position to the average.
    /// From these 4 points a normal direction can also be derived, which is used to orient the transform's rotation
    /// - This script is a prime example of how the Height Query System may be used.
    /// </summary>
    [ExecuteInEditMode]
    [AddComponentMenu("Stylized Water 3/Align Transform To Water")]
    public class AlignToWater : MonoBehaviour
    {
        //Because there two completely different methods of sampling the water's height, this Interface class provides a way to specify which is to be used.
        //In the case of the CPU-method, it contains everything else needed for such a query (eg. water level and material)
        public HeightQuerySystem.Interface heightInterface = new HeightQuerySystem.Interface();
        
        public Vector2 surfaceSize = new Vector2(0.5f, 0.5f);

        [Tooltip("Assign an optional transform to follow on the XZ axis.\n\nThis may be used if you want to use this component to read the water height at another transform's position.")]
        public Transform followTarget;
        public float heightOffset;
        [Range(0f, 8f)]
        [Tooltip("Controls how strongly the transform should rotate to align with the wave curvature")]
        public float rollAmount = 0.1f;
        [Tooltip("Add a Y-axis rotation to the transform. Note that this technically results in an incorrect alignment.")]
        [Range(0f, 360f)]
        public float rotation = 0f;

        [Tooltip("Smoothly blend towards the newly calculated position and rotation. May be used to combat jittering")]
        [Min(0f)]
        public float smoothing = 0.1f;
        
        private Vector3 normal;
        private float height;
        public enum HeightValue
        {
            Average,
            Maximum
        }
        public HeightValue heightValue;

        //A sampler defines:
		// • An array of world-space input positions to sample the height at.
		// • A float array (of equal size) that stores the output height values.
		// Samplers must always be initialized by specifying how many sample points are needed.
        private HeightQuerySystem.Sampler heightSampler;
        //A request ask the Height Query System to return the water height at every one of the sampler's positions from the GPU
        //It contains a callback event that needs to be subscribed to, to let you know when the request is completed
        private HeightQuerySystem.AsyncRequest heightRequest;

#pragma warning disable 108,114 //New keyword
        public Rigidbody rigidbody;
#pragma warning restore 108,114

        private Vector3 m_targetNormal;
        private float m_targetHeight;
        
#if UNITY_EDITOR
        public static bool EnableInEditor
        {
            get { return UnityEditor.EditorPrefs.GetBool("SW3_BUOYANCY_EDITOR_ENABLED", true); }
            set { UnityEditor.EditorPrefs.SetBool("SW3_BUOYANCY_EDITOR_ENABLED", value); }
        }
#endif

        private void Start()
        {
            rigidbody = GetComponent<Rigidbody>();
            
            if (rigidbody && rigidbody.useGravity)
            {
                Debug.LogWarning($"[Align Transform To Water] Disabled gravity on RigidBody \"{rigidbody.name}\". Otherwise its position can't be set without it struggling back");
                rigidbody.useGravity = false;
            }
        }
        
        private void OnEnable()
        {
#if UNITY_EDITOR
            if(Application.isPlaying == false) UnityEditor.SceneView.duringSceneGui += DuringSceneViewUpdate;
#endif

            //Sampler uses 4 sampling points, one for each corner of the rectangle/plane
            //In the context of a physics-based buoyancy system a sampler will use more than 4 sampling points
			heightSampler = new HeightQuerySystem.Sampler();
			heightSampler.SetSampleCount(4);
            
            //Only when using the GPU-readback method does the HeightQuerySystem come into play
            if (heightInterface.method == HeightQuerySystem.Interface.Method.GPU)
            {
                //Create a new request with the sampler created above.
				//Do this just once! It'll keep going until you call "Dispose()" on it.
                //Using the object's hashcode ensures the request gets a unique ID
                heightRequest = new HeightQuerySystem.AsyncRequest(this.GetHashCode(), heightSampler, this.name);
				
                //Issue the request so the system starts populating the "heightSampler" with data
                heightRequest.Issue();

                //True by default. This checks if the returned water height values are valid. If not, the value remains the same as last frame
                //This avoids objects that are not above any water surface, or fall outside the camera frustum, from falling down to -1000 heights
                heightRequest.invalidateMisses = true;
				
				//You may also call .Withdraw() if you wish to only temporarily remove the request from the system. Then call .Issue() again when needed.
                //This is also necessary when looking to call heightSampler.SetSampleCount() again.
                //Doing so avoids reallocating its resources but only temporarily takes it out of the running.
				
                //Subscribe to the event that indicates that data was retrieved from the GPU
                //Important to note that this may be a few (rendering) frames later than when the request was issued (at least 1)
                heightRequest.onCompleted += OnHeightRequestComplete;
            }
            else
            {
                prevHeight = this.transform.position.y;
                height = prevHeight;
            }
        }

        #if UNITY_EDITOR
        void DuringSceneViewUpdate(UnityEditor.SceneView sceneView)
        {
            FixedUpdate();
        }
        #endif

        private void OnHeightRequestComplete()
        {
            //Debug.Log("Height request returned");
            
            //Use the data. It pre-presents the water geometry's height value at each of the requested sample positions
            float xNeg = heightSampler.heightValues[0];
            float xPos = heightSampler.heightValues[1];
            float zNeg = heightSampler.heightValues[2];
            float zPos = heightSampler.heightValues[3];

            float newHeight = 0f;
            
            if (heightValue == HeightValue.Average)
            {
                newHeight = xNeg + xPos + zNeg + zPos;
                newHeight /= 4f;
            }
            if (heightValue == HeightValue.Maximum)
            {
                newHeight = Mathf.Max(Mathf.Max(xNeg, xPos), Mathf.Max(zNeg, zPos));
            }
            
            newHeight += heightOffset;         
            
            if (float.IsNaN(newHeight))
            {
				#if SWS_DEV
                Debug.LogError("Height is NaN");
				#endif
				
                //May occur during the first run
                newHeight = 0f;
            }

            if (heightInterface.method == HeightQuerySystem.Interface.Method.GPU)
            {
                //If all the samples were taking at a point where no water was visible, the height values would be -1000f.
                //Avoid setting an invalid height to prevent objects from sinking way down, instead keep them at the last valid height
                if (HeightQuerySystem.EqualsVoid(newHeight))
                {
                    UpdateSamplePositions();
                    return;
                }
            }

            height = newHeight;
            
            //Using 4 samples in a plus-shape pattern, a normal can be derived from the height differences
            normal = HeightQuerySystem.DeriveNormal(
                xNeg, xPos,
                zNeg, zPos,
                rollAmount);
            
            //Note: In a physics-based buoyancy system, calculating the normal will not be necessary. The varying upward forces will naturally align an object to the water.
            //Reading back the water surface normals directly will not be possible, since no such information is available (requires a deferred rendering water system)
            
            //Only updating the sampling positions here, for the next query. There would be no point in doing so for frames where the height request may not be returned
            UpdateSamplePositions();
        }
        
        private void Reset()
        {
            //Auto-assign water object if there is only one
            if (heightInterface.waterObject == null && WaterObject.Instances.Count > 0)
            {
                heightInterface.waterObject = WaterObject.Instances[0];
#if UNITY_EDITOR
                UnityEditor.EditorUtility.SetDirty(this);
#endif
            }
            
            AutoCalculateSurfaceSizeFromMeshes();
        }
        
        private void OnDisable()
        {
#if UNITY_EDITOR
            if(Application.isPlaying == false) UnityEditor.SceneView.duringSceneGui -= DuringSceneViewUpdate;
#endif
            
            if (heightRequest != null)
            {
                //Unsubscribe
                heightRequest.onCompleted -= OnHeightRequestComplete;
				
                //Important! Remove the request from the system so that it de-allocates the memory it uses.
                heightRequest.Dispose();
                heightRequest = null;
            }
            
            //Also dispose the height sampler. This stores two arrays that need to be de-allocated
            //Note: When using the GPU method, disposing 'heightRequest' will already do this!
            heightSampler.Dispose();
            heightSampler = null;
        }

        public void FixedUpdate()
        {
            if (!this.enabled) return;
            
#if UNITY_EDITOR
            if (!EnableInEditor && Application.isPlaying == false) return;
#endif
            
			//When using the CPU (wave pattern replication) method
            if (heightInterface.method == HeightQuerySystem.Interface.Method.CPU)
            {
                //The water object contains the material, and may be used to define a water level
                heightInterface.GetWaterObject(this.transform.position);

				//A reference to a Water Object is required, which in turn contains a material
                if (heightInterface.HasMissingReferences()) return;
                
                //This function reproduces the water shader's wave vertex animation.
                Gerstner.ComputeHeight(heightSampler, heightInterface);
                
				//At this point the 'heightSampler' has new height values, so can go right ahead and use them
                OnHeightRequestComplete();
                
                ApplyTransform();
            }
            else
            {
                ApplyTransform();
            }

        }

        //Translates the corners of the bounds to the 4 sampling points (plus-shaped pattern)
		//From local-space to world-space.
        private void UpdateSamplePositions()
        {
            //Note: Should the number of sample positions change in realtime, that creates an issue where resources have to be destroyed and recreated. Avoid doing so!
            //Should it be needed, do so by:
            //• heightRequest.Withdraw(); //Remove the active request for data
            //• heightSampler.SetSampleCount(count). //This will automatically reallocate the arrays if the count was changed.
            //• heightRequest.Issue(); //Re-issue the request with the new number of samples
            
            // ┤
            heightSampler.SetSamplePosition(0, ConvertToWorldSpace(new Vector3(-surfaceSize.x * 0.5f, 0, 0)));
            // ├
            heightSampler.SetSamplePosition(1, ConvertToWorldSpace(new Vector3(surfaceSize.x * 0.5f, 0, 0)));
            // ┬
            heightSampler.SetSamplePosition(2, ConvertToWorldSpace(new Vector3(0, 0, -surfaceSize.y * 0.5f)));
            // ┴
            heightSampler.SetSamplePosition(3, ConvertToWorldSpace(new Vector3(0, 0, surfaceSize.y * 0.5f)));
        }

        private Vector3 prevNormal = new Vector3(0, 1, 0);
        private float prevHeight;
        
        private void ApplyTransform()
        {
            //Smooth transition to new normal of this frame, particularly reduces jittering on rigid bodies
            if (smoothing > 0)
            {
                m_targetHeight = Mathf.Lerp(prevHeight, height, Time.deltaTime / smoothing);
                prevHeight = m_targetHeight;
                
                m_targetNormal = Vector3.Lerp(prevNormal, normal, Time.smoothDeltaTime / smoothing);
                prevNormal = m_targetNormal;
            }
            else
            {
                prevHeight = height;
                prevNormal = normal;

                m_targetHeight = height;
                m_targetNormal = normal;
            }

            var position = this.transform.position;
            if (followTarget)
            {
                position.x = followTarget.position.x;
                position.z = followTarget.position.z;
            }
            
            position.y = m_targetHeight;
            
            //Setting the normal of a transform directly overrides any and all external rotations
            //If the component is attached an object with a mesh, move the mesh into a child object and rotate that instead
            var newRotation = Quaternion.FromToRotation(Vector3.up, m_targetNormal);

            newRotation *= quaternion.RotateY(rotation * Mathf.Deg2Rad);
            if (Application.isPlaying && rigidbody)
            {
                if (rollAmount > 0)
                {
                    rigidbody.Move(position, newRotation);
                }
                else
                {
                    rigidbody.MovePosition(position);
                }
            }
            else
            {
                if (rollAmount > 0)
                {
                    this.transform.SetPositionAndRotation(position, newRotation);
                }
                else
                {
                    this.transform.position = position;
                }
            }
        }
        
        private Vector3 ConvertToWorldSpace(Vector3 position)
        {
            return this.transform.TransformPoint(position);
        }
        
        /// <summary>
        /// Automatically grab a starting point for the area size, based on the attached mesh(es)
        /// </summary>
        public void AutoCalculateSurfaceSizeFromMeshes()
        {
            MeshFilter[] meshFilters = GetComponentsInChildren<MeshFilter>();

            int meshCount = meshFilters.Length;

            if (meshCount > 0)
            {
                surfaceSize.x = 0f;
                surfaceSize.y = 0f;

                Bounds bounds = new Bounds();
                Vector3 minSum = Vector3.one * Mathf.Infinity;
                Vector3 maxSum = Vector3.one * Mathf.NegativeInfinity;
                
                for (int i = 0; i < meshCount; i++)
                {
                    minSum = Vector3.Min(minSum, meshFilters[i].sharedMesh.bounds.min);
                    maxSum = Vector3.Max(maxSum, meshFilters[i].sharedMesh.bounds.max);
                }
                bounds.SetMinMax(minSum, maxSum);
                
                surfaceSize.x = bounds.size.x;
                surfaceSize.y = bounds.size.z;
            }
        }
        
        private void OnDrawGizmosSelected()
        {
            if (heightSampler != null && heightSampler.IsCreated())
            {
                foreach (Vector3 p in heightSampler.positions)
                {
                    Gizmos.DrawWireCube(p - (Vector3.up * heightOffset), Vector3.one * 0.2f);
                }
            }

            Gizmos.DrawLine(this.transform.position, this.transform.position + (m_targetNormal * 2f));

            Gizmos.matrix = this.transform.localToWorldMatrix;
            Gizmos.DrawWireCube(Vector3.zero - (Vector3.up * heightOffset), new Vector3(surfaceSize.x, 0f, surfaceSize.y));
        }
    }
}