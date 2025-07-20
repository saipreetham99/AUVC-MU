// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

#ifndef WATER_REFLECTIONS_INCLUDED
#define WATER_REFLECTIONS_INCLUDED

#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DeclareDepthTexture.hlsl"
#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DeclareOpaqueTexture.hlsl"

#define AIR_RI 1.000293

//Schlick's BRDF fresnel
float ReflectionFresnel(float3 worldNormal, float3 viewDir, float exponent)
{
	float cosTheta = saturate(dot(worldNormal, viewDir));	
	return pow(max(0.0, AIR_RI - cosTheta), exponent);
}

float AttenuateSSR(float2 uv)
{
	float offset = min(1.0 - max(uv.x, uv.y), min(uv.x, uv.y));

	float result = offset / (0.1);
	result = saturate(result);

	return pow(result, 0.5);
}


float4 _WaterSSRParams;
//X: Enabled bool
//Y: Accept skybox hits

#define ALLOW_SSR _WaterSSRParams.x > 0.5
#define SSR_REFLECT_SKY _WaterSSRParams.y > 0.5

float4 _WaterSSRSettings;
//X: Steps
//Y: Step size
//Z: Max distance
//W: Thickness

#define SSR_SAMPLES _WaterSSRSettings.x
#define SSR_STEPSIZE _WaterSSRSettings.y
#define SSR_MAX_DISTANCE _WaterSSRSettings.z
#define SSR_THICKNESS _WaterSSRSettings.w

void RaymarchSSR(float3 positionVS, float3 direction, uint samples, half stepSize, half thickness, out half2 sampleUV, out half valid, out half outOfBounds)
{
	sampleUV = 0;
	valid = 0;
	outOfBounds = 0;

	direction *= stepSize;
	const half rcpStepCount = rcp(samples);
 
	UNITY_LOOP
	for(uint i = 0; i < samples; i++)
	{
		positionVS += direction;
		direction *= 1+stepSize;
		
		//View-space to screen-space UV
		sampleUV = ComputeNormalizedDeviceCoordinates(positionVS, GetViewToHClipMatrix());

		if (any(sampleUV < 0) || any(sampleUV > 1))
		{
			outOfBounds = 1;
			valid = 0;
			break;
		}
		
		outOfBounds = AttenuateSSR(sampleUV);

		//Sample Mip0, gradient sampling cannot work with loops
		float deviceDepth = SAMPLE_TEXTURE2D_X_LOD(_CameraDepthTexture, sampler_CameraDepthTexture, sampleUV, 0).r;

		//Depth is near-infinity. May want to reflect the skybox, if no reflection probes are present
		if(SSR_REFLECT_SKY && deviceDepth <= 0.00001)
		{
			valid = 1;
			continue;
		}
		
		//Calculate view-space position from UV and depth
		//Not using the ComputeViewSpacePosition function, since this negates the Z-component
		float3 samplePos = ComputeWorldSpacePosition(sampleUV, deviceDepth, UNITY_MATRIX_I_P);
		
		//Depth mismatch check. Geometry behind the water is invalid. If the difference in depth is large enough, consider it a miss.
		if (abs(samplePos.z - positionVS.z) > length(direction) * thickness) continue;
		
		if(samplePos.z > positionVS.z)
		{
			valid = 1;
			return;
		}
	}
}

TEXTURE2D_X(_PlanarReflection);
SAMPLER(sampler_PlanarReflection);

float3 SampleReflectionProbes(float3 reflectionVector, float3 positionWS, float smoothness, float2 screenPos)
{
	float3 probes = float3(0,0,0);
	
	probes = GlossyEnvironmentReflection(reflectionVector, positionWS, smoothness, 1.0, screenPos.xy).rgb;

	return probes;
}

float3 SampleReflections(float3 reflectionVector, float smoothness, float4 screenPos, float3 positionWS, float3 normalWS, float3 viewDir, float2 pixelOffset, bool planarReflectionsEnabled, bool ssrEnabled, out float3 renderedReflections)
{
	screenPos.xy += pixelOffset.xy * lerp(1.0, 0.1, unity_OrthoParams.w);
	screenPos /= screenPos.w;

	const float3 probes = SampleReflectionProbes(reflectionVector, positionWS, smoothness, screenPos.xy);
	
	float3 reflections = probes;

	//Output separately, for underwater rendering
	renderedReflections = 0;
	
	#if !_DISABLE_DEPTH_TEX
	if(ssrEnabled && ALLOW_SSR)
	{
		const float3 positionVS = TransformWorldToView(positionWS);
		const float3 direction = TransformWorldToViewDir(reflectionVector);

		float2 ssrUV = 0;
		half ssrRayMask, ssrEdgeMask = 0;

		RaymarchSSR(positionVS, direction, SSR_SAMPLES, SSR_STEPSIZE, SSR_THICKNESS, ssrUV, ssrRayMask, ssrEdgeMask);

		half ssrMask = ssrRayMask * ssrEdgeMask;
		const float3 reflectionSS = SampleSceneColor(ssrUV);

		reflections = lerp(reflections, reflectionSS, ssrMask);

		renderedReflections += reflectionSS * ssrMask;
	}
	#endif
		
	#if !_RIVER //Planar reflections are pointless on curved surfaces, skip
	if(planarReflectionsEnabled)
	{
		float4 planarReflections = SAMPLE_TEXTURE2D_X_LOD(_PlanarReflection, sampler_PlanarReflection, screenPos.xy, 0);
		//Terrain add-pass can output negative alpha values. Clamp as a safeguard against this
		planarReflections.a = saturate(planarReflections.a);
	
		reflections = lerp(reflections, planarReflections.rgb, planarReflections.a);

		renderedReflections = lerp(renderedReflections, planarReflections.rgb, planarReflections.a);
	}
	#endif
	
	return reflections;
}
#endif