#pragma once

#include "Cube.h"

namespace core {

class CubeUtil {

public:

	vec3f static calcDC(const PC_t::Ptr vCloud, const SCube& c);
	float static calcAGTV(const PC_t::Ptr vCloud, const SCube& c, int k);
	float static calcCubeSimilarity(const PC_t::Ptr vCloud, const SCube& a, const SCube& b);
	PC_t::Ptr static extractPtsFromCube(const PC_t::Ptr vCloud, const SCube& c);

};

}
