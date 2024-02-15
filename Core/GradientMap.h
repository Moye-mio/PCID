#pragma once

#include "BasicMath.h"
#include "Map.hpp"

namespace core {

	class CGradientMap : public CMap<vec2f> {
	public:
		CGradientMap();
		CGradientMap(uint vWidth, uint vHeight);
		CGradientMap(uint vWidth, uint vHeight, vec2f vValue);

	private:

	};

}