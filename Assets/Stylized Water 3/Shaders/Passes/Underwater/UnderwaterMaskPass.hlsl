    Pass
    {
        Name "Underwater Mask"
        Tags { "LightMode" = "Underwater" }

        HLSLPROGRAM

        #pragma multi_compile_instancing
        #pragma instancing_options renderinglayer
        
        #include_library "Libraries/URP.hlsl"
        #include_with_pragmas "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DOTS.hlsl"
        
        #define FULLSCREEN_QUAD
        #pragma shader_feature_local_vertex _WAVES
        
        //Multi-compile variants for installed extensions
        %multi_compile_vertex dynamic effects%

        #ifdef DYNAMIC_EFFECTS_ENABLED
        #include_library "DynamicEffects/DynamicEffects.hlsl"
        #endif

        #include_library "Underwater/UnderwaterMask.hlsl"

        #pragma vertex VertexWaterLine
        #pragma fragment frag
            
        half4 frag(UnderwaterMaskVaryings input) : SV_Target
        {
            UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);

            //Perform some antialiasing so the render target can be of a much lower resolution
            float gradient = pow(abs(input.uv.y), 256);
            return 1-gradient;
        }
        
        ENDHLSL
    }