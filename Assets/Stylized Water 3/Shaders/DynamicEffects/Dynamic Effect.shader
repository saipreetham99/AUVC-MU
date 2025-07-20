Shader "Stylized Water 3/Dynamic Effect/Default"
{
	Properties
	{
		[MaterialEnum(Off,0,Front,1,Back,2)] _Cull("Cull", Int) = 0
		_BaseMap ("Texture (R=Height mask, G=Foam Mask)", 2D) = "white" {}
		[Toggle] _RemapDisplacement ("Remap displacement", Float) = 0
		_AnimationSpeed ("Panning Speed (XY)", Vector) = (0,0,0,0)
		_MaskMap ("Mask", 2D) = "white" {}
		_MaskAnimationSpeed ("Panning Speed (XY)", Vector) = (0,0,0,0)

		[Header(Output)]
		
		_HeightScale ("Displacement scale", Float) = 0.1
		_FoamStrength ("Foam strength", Float) = 1.0
		_NormalStrength ("Normal strength", Float) = 1.0
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
		Cull [_Cull]
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
			
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "DynamicEffects.hlsl"
            #include_with_pragmas "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DOTS.hlsl"

			TEXTURE2D(_BaseMap);
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
			CBUFFER_END
			
			struct Attributes
			{
				float4 positionOS : POSITION;
				float2 uv : TEXCOORD0;
				float4 color : COLOR;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct Varyings
			{
				float2 uv : TEXCOORD0;
				float4 positionCS : SV_POSITION;
				float4 color : TEXCOORD1;
				float3 positionWS : TEXCOORD2;

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
				output.uv = input.uv;
				output.color = input.color;
				output.positionWS = TransformObjectToWorld(input.positionOS.xyz);

				return output;
			}
			
			float4 frag (Varyings input) : SV_Target
			{
				UNITY_SETUP_INSTANCE_ID(input);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);
				
				float4 col = SAMPLE_TEXTURE2D(_BaseMap, sampler_LinearRepeat, TRANSFORM_TEX(input.uv + (_TimeParameters.xx * _AnimationSpeed.xy), _BaseMap));
				
				float maskMap = SAMPLE_TEXTURE2D(_MaskMap, sampler_LinearRepeat, TRANSFORM_TEX(input.uv + (_TimeParameters.xx * _MaskAnimationSpeed.xy), _MaskMap)).r;

				float height = col.r * input.color.r;
				float foam = _FoamStrength * col.g * input.color.g;
				
				float alpha = input.color.a * col.a * maskMap;

				float displacement = height;
				if(_RemapDisplacement == 1)
				{
					displacement = displacement * 2.0 - 1.0;
				}

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