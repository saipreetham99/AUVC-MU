Shader "Stylized Water 3/Dynamic Effect/Waves"
{
	Properties
	{
		//Waves
		[WaveProfile] _WaveProfile("Wave Profile", 2D) = "black" {}
		[MaterialEnum(Mesh UV,0,World XZ projected,1)]_WorldSpaceUV("UV Coordinates", Float) = 1
		[Toggle(_TERRAIN_DEPTH)] _TerrainDepthOn("Use terrain depth", Float) = 0
		_TerrainDepthFalloff("Terrain depth falloff", Float) = 1

		_WaveSpeed("Speed", Float) = 2
		_WaveHeight("Height", Range(0 , 10)) = 0.25
		[IntSlider] _WaveMaxLayers("Count", Range(1 , 64)) = 1
		_WaveDirection("Direction", vector) = (1,1,1,1)
		_WaveFrequency("Frequency", Float) = 1
		
		//_BaseMap ("Texture (R=Height mask, G=Foam Mask)", 2D) = "white" {}
		[Toggle] _RemapDisplacement ("Remap displacement", Float) = 0
		//_AnimationSpeed ("Panning Speed (XY)", Vector) = (0,0,0,0)
		_MaskMap ("Mask", 2D) = "white" {}
		//_MaskAnimationSpeed ("Panning Speed (XY)", Vector) = (0,0,0,0)
		_FoamCapThreshold("Foam Cap Threshold", Range(0 , 1)) = 1

		[Header(Output)]
		
		[PerRendererData] _HeightScale ("Displacement scale", Float) = 1.0
		[PerRendererData] _FoamStrength ("Foam strength", Float) = 1.0
		[PerRendererData] _NormalStrength ("Normal strength", Float) = 1.0
	}
	
	SubShader
	{
		Tags 
		{ 
			"LightMode" = "WaterDynamicEffect"
			//"LightMode" = "UniversalForward" //Uncomment to enable regular rendering
			"RenderType" = "Transparent"
			"RenderQueue" = "Transparent"
		}

		Blend SrcAlpha One
		BlendOp Add
		ZWrite Off
		//ZClip Off
		//ZTest Always
		
		Pass
		{
			Name "Output"
			
			HLSLPROGRAM
			#pragma vertex vert
			#pragma fragment frag

			#pragma multi_compile_instancing
			//#pragma instancing_options procedural:ParticleInstancingSetup
			//#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ParticlesInstancing.hlsl"

			#pragma shader_feature_local_fragment _TERRAIN_DEPTH
			
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "DynamicEffects.hlsl"
			
			TEXTURE2D(_MaskMap);
			
			CBUFFER_START(UnityPerMaterial)
				float4 _BaseMap_ST;
				float4 _MaskMap_ST;
				float2 _AnimationSpeed;
				float2 _MaskAnimationSpeed;
				float _HeightScale;
				float _FoamStrength;
				float _NormalStrength;
				float _RemapDisplacement;

				float _WaveNormalStr;
				float _WaveHeight;
				float _WaveFrequency;
				float _WaveDistance;
				float _WaveSteepness;
				float _WaveSpeed;
				uint _WaveMaxLayers;
				float4 _WaveDirection;
				float4 _WaveProfile_TexelSize;
				float _FoamCapThreshold;
				bool _WorldSpaceUV;
				half _TerrainDepthFalloff;
			CBUFFER_END

			#include "../Libraries/Common.hlsl"
			#include "../Libraries/Waves.hlsl"
            //Not needed, but registers it as a dependency
			#include "../Libraries/Gerstner.hlsl"

			#if _TERRAIN_DEPTH
			#include "../Libraries/Terrain.hlsl"
			#endif			
			
			struct Attributes
			{
				float4 positionOS : POSITION;
				float2 uv : TEXCOORD0;
				float4 color : COLOR;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct Varyings
			{
				float4 uv : TEXCOORD0;
				float4 positionCS : SV_POSITION;
				float3 positionWS : TEXCOORD2;
				float4 color : TEXCOORD1;

				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};
			
			Varyings vert (Attributes input)
			{
				Varyings output;
				
				UNITY_SETUP_INSTANCE_ID(input);
				UNITY_TRANSFER_INSTANCE_ID(input, output);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);

				output.positionCS = TransformObjectToHClip(input.positionOS.xyz);
				output.positionWS = TransformObjectToWorld(input.positionOS.xyz);
				
				output.uv.xy = input.uv;
				output.uv.z = _TimeParameters.x;
				output.uv.w = 0;
				
				output.color = input.color;

				return output;
			}
			
			float4 frag (Varyings input) : SV_Target
			{
				UNITY_SETUP_INSTANCE_ID(input);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);

				float3 waveOffset = float3(0,0,0);
				float3 waveTangent = waveOffset;
				float3 waveBiTangent = waveOffset;

				float2 waveDir = _WaveDirection.xy;

				#if _RIVER
				waveDir.x = 1;
				waveDir.y = -1;
				#endif

				//float3 positionWS = input.uv.xyy * 100;
				float2 uv = _WorldSpaceUV ? input.positionWS.xz : input.uv.xy;

				float terrainDepth = 1;
				#if _TERRAIN_DEPTH
				terrainDepth = SampleTerrainHeight(input.positionWS);
				uv.x = 0;
				uv.y = terrainDepth;
				#endif
				
				CalculateGerstnerWaves_float(_WaveProfile, _WaveProfile_TexelSize.z, uv,_WaveFrequency, (TIME_FRAG_INPUT) * _WaveSpeed, _WaveNormalStr, waveDir, _WaveMaxLayers,
					/* out */ waveOffset, waveTangent, waveBiTangent);

				float waveHeight = waveOffset.y * _WaveHeight;

				float maskMap = SAMPLE_TEXTURE2D(_MaskMap, sampler_LinearClamp, TRANSFORM_TEX(input.uv, _MaskMap)).r;

				float height = waveHeight * input.color.r;
				float displacement = height;
				if(_RemapDisplacement == 1)
				{
					//displacement = displacement * 2.0 - 1.0;
				}
				float foam = _FoamStrength * smoothstep(1-_FoamCapThreshold, 1-_FoamCapThreshold + 1.0, displacement) * input.color.g;
				
				float alpha = input.color.a * maskMap;


				EffectOutput output;
				output.displacement = displacement * _HeightScale * alpha;
				output.foamAmount = foam * alpha;
				output.normalGradient = displacement * _NormalStrength * alpha;
				output.alpha = 1;
				
				OUTPUT_EFFECT(output, input.positionWS);
			}
			ENDHLSL
		}
	}
}