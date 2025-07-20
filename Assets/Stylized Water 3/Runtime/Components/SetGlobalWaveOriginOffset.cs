using System;
using UnityEngine;

namespace StylizedWater3
{
    [ExecuteAlways]
    [AddComponentMenu("Stylized Water 3/Global Wave Origin Offset")]
    public class SetGlobalWaveOriginOffset : MonoBehaviour
    {
        private readonly int _GlobalWaveOriginOffset = Shader.PropertyToID("_GlobalWaveOriginOffset");

        private void Update()
        {
            Shader.SetGlobalVector(_GlobalWaveOriginOffset, this.transform.position);
        }

        private void OnDisable()
        {
            Shader.SetGlobalVector(_GlobalWaveOriginOffset, Vector3.zero);
        }
    }
}