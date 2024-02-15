#include "pch.h"
#include "GradientMap.h"

using namespace core;

core::CGradientMap::CGradientMap()
	: CMap<vec2f>(0, 0, vec2f { -FLT_MAX, -FLT_MAX }) {
	m_Empty = vec2f { -FLT_MAX, -FLT_MAX };
}

core::CGradientMap::CGradientMap(uint vWidth, uint vHeight)
	: CMap<vec2f>(vWidth, vHeight, vec2f { -FLT_MAX, -FLT_MAX }) {
	m_Empty = vec2f { -FLT_MAX, -FLT_MAX };
}

core::CGradientMap::CGradientMap(uint vWidth, uint vHeight, vec2f vValue)
	: CMap<vec2f>(vWidth, vHeight, vValue) {
	m_Empty = vec2f { -FLT_MAX, -FLT_MAX };
}