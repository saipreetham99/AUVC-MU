// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

#ifndef UNDERWATER_FOG_INCLUDED
#define UNDERWATER_FOG_INCLUDED

#include "UnderwaterMask.hlsl"
#include "../Libraries/Lighting.hlsl"

float _StartDistance;
float _UnderwaterFogDensity;

float4 _UnderwaterHeightFogParams;
//X: Height start
//Y: Height end (depth)
//Z: Density
//W: Brightness

float _UnderwaterFogBrightness;
float _UnderwaterColorAbsorption;

/////////////
// DENSITY //
/////////////

//Radial distance
float ComputeDistanceXYZ(float3 positionWS, float startDistance, float distanceDensity)
{
	//positionWS.y = min(positionWS.y, SampleWaterLevel(positionWS));
	
	float attenuation = length(_WorldSpaceCameraPos.xyz - positionWS.xyz);

	float3 viewDir = normalize(positionWS - _WorldSpaceCameraPos);
	half horizon = 1-saturate(viewDir.y);

	//Attenuate the fog by the vertical viewing angle. This creates a clear cylinder around the camera
	horizon = smoothstep(0.2, 1, horizon);
	//attenuation *= horizon;
	
	//Start distance
	attenuation -= _ProjectionParams.y + startDistance;
	attenuation *= distanceDensity;
	
	return 1-(exp(-attenuation));
}

float ComputeDistanceXYZ(float3 positionWS)
{
	return ComputeDistanceXYZ(positionWS, _StartDistance, _UnderwaterFogDensity);
}

float ComputeUnderwaterFogHeight(float3 positionWS, float waterHeight, float startHeight, float endHeight, float density)
{
	float start = ((waterHeight-startHeight) - 1.0 - endHeight);
	
	float3 viewDir = (_WorldSpaceCameraPos.xyz - positionWS);

	float FdotC = _WorldSpaceCameraPos.y - start; //Camera/fog plane height difference
	float k = (FdotC <= 0.0f ? 1.0f : 0.0f); //Is camera below height fog
	float FdotP = positionWS.y - start;
	float FdotV = viewDir.y;
	
	float c1 = k * (FdotP + FdotC);
	float c2 = (1 - 2.0 * k) * FdotP;
	float g = min(c2, 0.0);
	g = -density * (c1 - (g * g) / abs(FdotV + 1e-5));
	
	return 1-exp(-g);	
}

float ComputeUnderwaterFogHeight(float3 positionWS)
{
	return ComputeUnderwaterFogHeight(positionWS, _WaterLevel, _UnderwaterHeightFogParams.x, _UnderwaterHeightFogParams.y, _UnderwaterHeightFogParams.z);
}

float ComputeDensity(float distanceDepth, float heightDepth)
{
	//Blend of both factors
	return saturate(distanceDepth + heightDepth);
}

float GetUnderwaterFogDensity(float3 positionWS)
{
	const float distanceDensity = ComputeDistanceXYZ(positionWS);
	const float heightDensity = ComputeUnderwaterFogHeight(positionWS);
	const float density = ComputeDensity(distanceDensity, heightDensity);

	return density;
}

void GetWaterDensity_float(float3 positionWS, out float density)
{
	density = GetUnderwaterFogDensity(positionWS.xyz);
}

///////////
// COLOR //
///////////

float4 GetUnderwaterFogColor(float4 shallow, float4 deep, float distanceDensity, float heightDensity)
{
	float4 waterColor = 0;

	//waterColor.rgb = lerp(shallow.rgb, deep.rgb, saturate(distanceDensity * 1.0)) * _UnderwaterFogBrightness;
	waterColor.rgb = deep.rgb * _UnderwaterFogBrightness;

	float density = ComputeDensity(distanceDensity, heightDensity);
	waterColor.a = density;

	if(_UnderwaterColorAbsorption > 0)
	{
		float scatterAmount = LightAbsorption(_UnderwaterColorAbsorption, distanceDensity);
		waterColor.rgb *= scatterAmount;
	}
	
	waterColor.rgb = lerp(waterColor.rgb, deep.rgb * _UnderwaterHeightFogParams.w, heightDensity);
	
	return waterColor;
}
#endif