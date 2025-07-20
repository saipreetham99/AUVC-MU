// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Object = UnityEngine.Object;

namespace StylizedWater3
{
    public static partial class HeightQuerySystem
    {
        /// <summary>
        /// Holds a list of sampling positions, and the returned water height values relative to them
        /// </summary>
        public class Sampler
        {
            /// <summary>
            /// Input sample positions in world-space
            /// </summary>
            public NativeArray<float3> positions;
            /// <summary>
            /// Output height values at each sampling <see cref="positions"/>
            /// </summary>
            public NativeArray<float> heightValues;

            private int currentSampleCount = 0;

            /// <summary>
            /// Checks if the sampler has been initialized
            /// </summary>
            /// <returns></returns>
            public bool IsCreated()
            {
                return currentSampleCount > 0;
            }

            public int SampleCount => currentSampleCount;

            /// <summary>
            /// Initialize the sampler with a number of sampling points. This allocates the memory required.
            /// </summary>
            /// <param name="sampleCount">Number of positions to sample at. For best performance, this number should be conservative!</param>
            /// <param name="cpu">Specify if this sampler is used with the CPU-height query method. If so, no limit is imposed on the sample count</param>
            /// <exception cref="Exception"></exception>
            public void SetSampleCount(int sampleCount, bool cpu = false)
            {
                if (cpu == false && sampleCount > Query.MAX_SIZE)
                {
                    Dispose();

                    throw new Exception($"The number of sample positions ({sampleCount}) exceeds the maximum capacity ({Query.MAX_SIZE}) of a single sampler." +
                                        $" Decrease the number of input positions, or issue multiple smaller requests");
                }

                //Changed
                if (currentSampleCount != sampleCount)
                {
                    #if SWS_DEV
                    if(currentSampleCount > 0) Debug.Log($"Sampler count changed from {currentSampleCount} to {sampleCount}");
                    #endif
                    
                    if (positions.IsCreated) positions.Dispose();

                    //Input data
                    positions = new NativeArray<float3>(sampleCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

                    if (heightValues.IsCreated) heightValues.Dispose();
                    
                    //Output data
                    heightValues = new NativeArray<float>(sampleCount, Allocator.Persistent);
                }
                currentSampleCount = sampleCount;
            }
            
            [Obsolete("Use SetSampleCount() instead. Method was renamed for clarity")]
            public void Initialize(int sampleCount, bool cpu = false)
            {
                SetSampleCount(sampleCount, cpu);
            }

            public void SetSamplePosition(int index, float3 value)
            {
                if (index > currentSampleCount)
                {
                    throw new Exception($"Index out of range. This sampler was initialized with {currentSampleCount} number of samples. Dispose() and SetSampleCount() the sampler to increase the number of samples!");
                }

                positions[index] = value;
            }

            public void Dispose()
            {
                if (positions.IsCreated) positions.Dispose();
                if (heightValues.IsCreated) heightValues.Dispose();
                
                currentSampleCount = 0;
            }
        }
    }
}