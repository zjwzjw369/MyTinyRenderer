#pragma once
#include "tgaimage.h"
#include "geometry.h"



struct IShader
{
	struct VSIN
	{
		vec4 gl_vertex;
		vec4 uv0;
	} vs_in;

	struct VSOUT
	{
		vec4 gl_vertex;
		vec4 uv0;
	} vs_out;

	virtual void vs_out_to_ps_in(vec3 bar, bool interpolate=true) = 0;
	virtual vec4 vertex(int nthvert) = 0;
	virtual bool fragment(TGAColor &color) = 0;
};
