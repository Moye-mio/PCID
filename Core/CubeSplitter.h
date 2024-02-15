#pragma once

#include "Cube.h"

namespace core {

class CCubeSplitter {

public:

	bool run(const PC_t::Ptr vCloud, const vec3f& vSplitterOrigin, float vCubeLength);
	void dumpCubes(std::vector<SCube>& vioCubes) { vioCubes = m_Cubes; }


private:
	vec3i __calcPoint2Cube(const Point_t& vPt, const vec3f& vSplitterOrigin, float vCubeLength);


private:
	std::vector<SCube> m_Cubes;


};

}

