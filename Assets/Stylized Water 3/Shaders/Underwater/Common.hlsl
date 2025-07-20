#ifndef UNDERWATER_COMMON_INCLUDED
#define UNDERWATER_COMMON_INCLUDED

bool _UnderwaterRenderingEnabled;
bool _FullySubmerged;

float _WaterLevel;
float _ClipOffset;

#if !defined(SHADERGRAPH_PREVIEW)
TEXTURE2D(_UnderwaterMask);
SAMPLER(sampler_UnderwaterMask);
#endif

#endif