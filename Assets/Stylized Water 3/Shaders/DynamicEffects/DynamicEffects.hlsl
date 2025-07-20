// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

#define SW_DE_VERSION 301

#if !defined(SHADERGRAPH_PREVIEW)
#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
#else
SamplerState sampler_LinearClamp;
#endif

TEXTURE2D(_WaterDynamicEffectsBuffer);
float4 _WaterDynamicEffectsBuffer_TexelSize;
TEXTURE2D(_WaterDynamicEffectsNormals);

float4 _WaterDynamicEffectsCoords;
//XY: Bounds min position
//Z: Bounds scale
//W: Bool, rendering pass active
float4 _WaterDynamicEffectsParams;
//X: Enable normals
//Y: Enable displacement
//Z: Render range
//W: End fade distance

#define NORMALS_AVAILABLE _WaterDynamicEffectsParams.x > 0
#define GLOBAL_DISPLACEMENT_STRENGTH _WaterDynamicEffectsParams.y

#define DE_HEIGHT_CHANNEL 0
#define DE_FOAM_CHANNEL 1
#define DE_NORMALS_CHANNEL 2
#define DE_ALPHA_CHANNEL 3

#include "../Libraries/Projection.hlsl"
float DynamicEffectsBoundsEdgeMask(float3 position)
{
	return ProjectionEdgeMask(position, _WaterDynamicEffectsCoords.xy, _WaterDynamicEffectsCoords.z, _WaterDynamicEffectsParams.w);
}

//Shader Graph
void BoundsEdgeMask_float(float3 positionWS, out float mask)
{
	mask = DynamicEffectsBoundsEdgeMask(positionWS);
}

float2 DynamicEffectsSampleCoords(float3 positionWS)
{
	return WorldToProjectionUV(positionWS, _WaterDynamicEffectsCoords.xy, _WaterDynamicEffectsCoords.z);
}

//Account for the SampleDynamicEffectsData being called in a vertex or tessellation shader
#if defined(SHADER_STAGE_VERTEX) || defined(SHADER_STAGE_DOMAIN) || defined(SHADER_STAGE_HULL)
#define SAMPLE_FUNC(texName, sampler, uv) SAMPLE_TEXTURE2D_LOD(texName, sampler, uv, 0)
#else
#define SAMPLE_FUNC(texName, sampler, uv) SAMPLE_TEXTURE2D(texName, sampler, uv)
#endif

float4 SampleDynamicEffectsData(float3 positionWS)
{
	float4 data = 0;
	
	if(_WaterDynamicEffectsCoords.w > 0)
	{
		data = SAMPLE_FUNC(_WaterDynamicEffectsBuffer, sampler_LinearClamp, DynamicEffectsSampleCoords(positionWS));
				
		data[DE_HEIGHT_CHANNEL] *= GLOBAL_DISPLACEMENT_STRENGTH;
	}
	
	return data;
}

//Shorthand for only sampling displacement
float SampleDynamicEffectsDisplacement(float3 positionWS)
{
	return SampleDynamicEffectsData(positionWS).r;
}

//Shader Graph
void SampleDynamicEffectsData_float(float3 positionWS, out float4 data)
{
	data = SampleDynamicEffectsData(positionWS);
}

float4 SampleDynamicEffectsNormals(float3 positionWS)
{
	float4 neutralNormal = float4(0, 1.0, 0.0, 0.0);
	float4 normals = neutralNormal;
	
	if(_WaterDynamicEffectsCoords.w > 0 && NORMALS_AVAILABLE)
	{
		//Buffer contains X and Z normal vectors packed into the RG channels
		normals = SAMPLE_TEXTURE2D(_WaterDynamicEffectsNormals, sampler_LinearClamp, DynamicEffectsSampleCoords(positionWS));

		//Issue when mipmaps are enabled, streaking occurs at edges still
		float edgeMask = DynamicEffectsBoundsEdgeMask(positionWS);

		normals.xz = normals.xy * 2.0 - 1.0;
		//Reconstruct Y-value
		normals.y = max(1.0e-16, sqrt(1.0 - saturate(dot(normals.xz, normals.xz))));
		
		normals = lerp(neutralNormal, normals, edgeMask);

		normals.a = length(normals.xz) * edgeMask;
	}
	
	return normals;
}

#include "../Libraries/Foam.hlsl"
TEXTURE2D(_FoamTexDynamic);

float2 SampleDynamicFoam(float2 uv, float tiling, float subTiling, float2 time, float speed, float subSpeed)
{
	return SampleFoamLayer(TEXTURE2D_ARGS(_FoamTexDynamic, sampler_FoamTex), uv, tiling, time, speed, subTiling, subSpeed);
}

struct EffectOutput
{
	float displacement;
	float foamAmount;
	float normalGradient;
	float alpha;
};

float4 OutputEffect(float3 positionWS, float height, float foam, float normalHeight, float alpha)
{
	//Apply rendering boundary edge fade
	float edgeMask = DynamicEffectsBoundsEdgeMask(positionWS);

	height *= edgeMask;
	foam *= edgeMask;
	normalHeight *= edgeMask;
	alpha *= edgeMask;
	
	return float4(height, foam, normalHeight, alpha);

}
#define OUTPUT_EFFECT(struct, positionWS) return OutputEffect(positionWS, struct.displacement, struct.foamAmount, struct.normalGradient, struct.alpha)