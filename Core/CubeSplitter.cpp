#include "pch.h"
#include "CubeSplitter.h"

using namespace core;

bool CCubeSplitter::run(const PC_t::Ptr vCloud, const vec3f& vSplitterOrigin, float vCubeLength) {
	_EARLY_RETURN(!isPointCloudValid(vCloud), "cubesplitter: cloud is invalid.", false);
	
	std::unordered_map<std::string, SCube> um;

	for (int i = 0; i < vCloud->size(); i++) {
		vec3i CubeId = __calcPoint2Cube(vCloud->at(i), vSplitterOrigin, vCubeLength);
		std::string CubeKey = std::to_string(CubeId.x) + "," + std::to_string(CubeId.y) + "," + std::to_string(CubeId.z);

		if (auto it = um.find(CubeKey); it != um.end()) {
			it->second.indices.push_back(i);
		}
		else {
			SCube Cube;
			Cube.id = CubeId;
			Cube.box = SAABB { vSplitterOrigin.x + vCubeLength * CubeId.x, vSplitterOrigin.y + vCubeLength * CubeId.y, vSplitterOrigin.z + vCubeLength * CubeId.z, 
							   vSplitterOrigin.x + vCubeLength * (CubeId.x + 1), vSplitterOrigin.y + vCubeLength * (CubeId.y + 1), vSplitterOrigin.z + vCubeLength * (CubeId.z + 1)};
			Cube.indices.push_back(i);
			um.insert(std::make_pair(CubeKey, Cube));
		}
	}
	
	for (const auto& e : um) {
		m_Cubes.emplace_back(e.second);
	}

	std::cout << "cube splitter: generate " << m_Cubes.size() << " cubes" << std::endl;

	return true;
}

vec3i CCubeSplitter::__calcPoint2Cube(const Point_t& vPt, const vec3f& vSplitterOrigin, float vCubeLength) {
	int x = (vPt.x - vSplitterOrigin.x) / vCubeLength;
	int y = (vPt.y - vSplitterOrigin.y) / vCubeLength;
	int z = (vPt.z - vSplitterOrigin.z) / vCubeLength;

	return vec3i { x, y, z };
}
