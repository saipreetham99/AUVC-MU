// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

#ifndef UNDERWATER_MASK_INCLUDED
#define UNDERWATER_MASK_INCLUDED

#ifndef SHADERGRAPH_PREVIEW
#include "../Libraries/URP.hlsl"
#include "../Libraries/Input.hlsl"
#include "../Libraries/Common.hlsl"
//#include "../Libraries/Height.hlsl"
#include "../Libraries/Waves.hlsl"
#endif
#include "../Underwater/Common.hlsl"

float _WaterLineWidth;

//Additional wave height to avoid any cracks
#define PADDING 0.0

//Enabled when inspecting the mesh on a renderer for a specific camera
//#define DEBUG

#ifdef DEBUG
float _NearPlane;
float _FarPlane;
float _CamFov;
float3 _CamForward;
float3 _CampUp;
float3 _CamRight;
float3 _CamPos;
float _CamAspect;

#define NEAR_PLANE _NearPlane
#define ASPECT _CamAspect
#define CAM_FOV _CamFov
#define CAM_POS _CamPos
#define CAM_RIGHT _CamRight
#define CAM_UP _CampUp
#define CAM_FWD _CamForward
#else
//Current camera
#define NEAR_PLANE _ProjectionParams.y
#define ASPECT _ScreenParams.x / _ScreenParams.y
#define CAM_FOV unity_CameraInvProjection._m11
#define CAM_POS _WorldSpaceCameraPos
#define CAM_RIGHT unity_WorldToCamera[0].xyz //Possibly flipped as well, but doesn't matter
#define CAM_UP unity_WorldToCamera[1].xyz
//The array variant is flipped when stereo rendering is in use. Using the camera center forward vector also works for VR
#define CAM_FWD -GetWorldToViewMatrix()[2].xyz
#endif

#define DEGREES_2_RAD PI / 180.0
#define FOV_SCALAR 2.0
#define DEGREE2RAD 0.017453292f

 
struct UnderwaterMaskAttributes
{
	float4 positionOS : POSITION;
	float2 uv         : TEXCOORD0;
	float4 color       : COLOR;
	UNITY_VERTEX_INPUT_INSTANCE_ID
};

struct UnderwaterMaskVaryings
{
	float4 positionCS 	: SV_POSITION;
	float3 uv 			: TEXCOORD0;
	float3 positionWS	: TEXCOORD1;
	float4 screenPos 	: TEXCOORD2;
	UNITY_VERTEX_OUTPUT_STEREO
};

float SampleWaterLevel(float3 positionWS)
{
	return _WaterLevel;
}

void SampleWaterLevel_float(float3 positionWS, out float height)
{
	height = SampleWaterLevel(positionWS);
}

#ifndef SHADERGRAPH_PREVIEW
UnderwaterMaskVaryings VertexWaterLine(UnderwaterMaskAttributes input)
{
	UnderwaterMaskVaryings output = (UnderwaterMaskVaryings)0;

	UNITY_SETUP_INSTANCE_ID(input);
	UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);

	output.uv.xy = input.uv.xy;
	output.uv.z = _TimeParameters.x;

	float clipPlane = (NEAR_PLANE * 2.0) + _ClipOffset;
	
	#if _WATERLINE
	//Scale unit rectangle by width
	input.positionOS.y *= _WaterLineWidth;
	#endif

	//Near-clip plane position
	float3 nearPlaneCenter = CAM_POS + (CAM_FWD * clipPlane);

	const float fovScalar = FOV_SCALAR * CAM_FOV;
	const float aspectRatio = ASPECT;

	//In VR, at the far edges of the headset (difficult to see) the mesh cuts off a little.
	//Compensate by widening the mesh a little
	#if defined(USING_STEREO_MATRICES)
	input.positionOS.x *= 1.1;
	#endif
	
	//Position vertices on the near-clip plane and scale to fit
	float3 positionWS = (nearPlaneCenter + (CAM_RIGHT * (input.positionOS.x * aspectRatio * fovScalar) * clipPlane)) + (CAM_UP * input.positionOS.y * clipPlane * fovScalar);
	float3 bottom = (nearPlaneCenter + (CAM_RIGHT * (input.positionOS.x * aspectRatio * fovScalar) * clipPlane)) - (CAM_UP * clipPlane * fovScalar);
	float3 top = (nearPlaneCenter + (CAM_RIGHT * (input.positionOS.x * aspectRatio * fovScalar) * clipPlane)) + (CAM_UP * clipPlane * fovScalar);

	float planeLength = distance(bottom, top);
	
	float waterHeight = SampleWaterLevel(positionWS);

	#if _WAVES
	float3 waveOffset = float3(0,0,0);
	float3 waveNormal = float3(0,1,0);

	CalculateWaves(_WaveProfile, _WaveProfile_TexelSize.z, _WaveMaxLayers, positionWS.xz, _WaveFrequency, positionWS.xyz, _Direction.xy, float3(0,1,0), (TIME_VERTEX * _Speed) * _WaveSpeed, 0.0, float3(_WaveSteepness, _WaveHeight, _WaveSteepness),
_WaveNormalStr, _WaveFadeDistance.x, _WaveFadeDistance.y,
//Out
waveOffset, waveNormal);

	waterHeight += waveOffset.y;
	#endif
	
	#if DYNAMIC_EFFECTS_ENABLED
	waterHeight += SampleDynamicEffectsDisplacement(positionWS.xyz);
	#endif

	//Test
	//waterHeight += sin(input.positionOS.x * 32 + output.uv.z) * .05;
	//waterHeight += sin(-input.positionOS.x * 16 - (output.uv.z * 2)) * .1;
	
	//Distance from near-clip bottom to water level (straight up)
	float depth = waterHeight - bottom.y;

	//if(HasHitWaterSurface(waterHeight))
	{
		//Camera's X-angle
		float upFactor = dot(CAM_UP, float3(0.0, 1.0, 0.0));
		float angle = (acos(upFactor) * 180.0) / PI;

		//Distance from center to water level along tangent (from known opposite length and angle)
		float hypotenuse = (depth / cos(DEGREES_2_RAD * angle));

		//Intersection point with water level when traveling along the plane's tangent
		float3 samplePos = bottom + (CAM_UP * hypotenuse);
		
		//Length between the bottom position and the water surface
		float delta = length(samplePos - bottom);
	
		//If the bottom of the screen is submerged
		if(bottom.y <= waterHeight)
		{
			#if _WATERLINE
			//Align to vertical center
			delta -= planeLength * 0.5;

			//Position onto water line
			positionWS += (CAM_UP * delta);

			//Full-screen quad
			#else
		
			if(input.positionOS.y >= 0.5)
			{	   				
				positionWS = bottom + (CAM_UP * delta);
			}
		
			#endif
		}
		//Collapse the vertices, otherwise the top goes down through the bottom
		else
		{
			//Collapse vertices
			positionWS = bottom;
		}
	}


	output.positionCS = TransformWorldToHClip(positionWS);

	//Test rendering as original mesh
	//input.positionOS.y = waterHeight;
	//output.positionCS = TransformObjectToHClip(input.positionOS);
	
	output.positionWS = positionWS;
	output.screenPos = ComputeScreenPos(output.positionCS);

   return output;
}
#endif

float SampleUnderwaterMask(float2 screenPos)
{
	#ifndef SHADERGRAPH_PREVIEW //SAMPLE_TEXTURE2D_X is yet available

	if(_UnderwaterRenderingEnabled)
	{
		if(_FullySubmerged)
		{
			return 1;
		}
		else
		{
			return SAMPLE_TEXTURE2D(_UnderwaterMask, sampler_UnderwaterMask, screenPos.xy).r;
		}
	}
	else
	{
		return 0;
	}
	#else
	return 0;
	#endif
}

// Shader Graph //
void SampleUnderwaterMask_float(float4 screenPos, out float mask)
{
	mask = SampleUnderwaterMask(screenPos.xy / screenPos.w);
}

//Clip the water using a fake near-clipping plane.
float ClipSurface(float4 screenPos, float3 positionWS, float3 positionCS, float vFace)
{
	#if UNDERWATER_ENABLED && !defined(SHADERGRAPH_PREVIEW) && !defined(UNITY_GRAPHFUNCTIONS_LW_INCLUDED)
	
	const float clipStart = NEAR_PLANE + _ClipOffset;
	const float3 viewPos = TransformWorldToView(positionWS);

	//Distance based scalar
	float f = saturate(-viewPos.z / clipStart);
	float mask = floor(f);

	//Clip space depth is not enough since vertex density is likely lower than the underwater mask
	//Sample the per-pixel water mask
	const float underwaterMask = SampleUnderwaterMask(screenPos.xy / screenPos.w);
	mask *= lerp(underwaterMask, 1-underwaterMask, vFace);
	
	clip(mask - 0.5);

	return mask;
	#else
	return 1.0;
#endif
}
#endif