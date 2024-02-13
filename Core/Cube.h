#pragma once

#include "AABB.h"

namespace core {

struct SCube {
	std::vector<int> indices;
	SAABB box;
	vec3i id;
	int sourceid;
	bool ishole = false;
};

}


