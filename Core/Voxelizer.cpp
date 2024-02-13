#include "pch.h"
#include "Voxelizer.h"
#include "PointCloudUtil.h"
#include "DensityEstimator.h"

using namespace core;

PC_t::Ptr CVoxelizer::run(const PC_t::Ptr vCloud) {
	_EARLY_RETURN(!isPointCloudValid(vCloud), "voxelizer: input cloud in invalid", nullptr);
	
	CDensityEstimator DE;
	m_Scale = DE.run(vCloud, 1) * 0.6f;

	for (auto& p : *vCloud) {
		p.x /= m_Scale;
		p.y /= m_Scale;
		p.z /= m_Scale;
	}
	
	PC_t::Ptr pVoxels(new PC_t);
	std::unordered_set<std::string> us;
	for (auto& p : *vCloud) {
		Point_t e = __calcNearestCenter(p);
		vec3i Key {e.x, e.y, e.z};
		std::string KeyString(std::to_string(Key.x) + "," + std::to_string(Key.y) + "," + std::to_string(Key.z));
		if (auto it = us.find(KeyString); it != us.end()) {
			continue;
		}
		else {
			us.insert(KeyString);
			pVoxels->emplace_back(e);
		}
	}

	std::cout << "Voxelizer: size " << pVoxels->size() << std::endl;
	
	return pVoxels;
}

Point_t CVoxelizer::__calcNearestCenter(const Point_t& vPt) {
	std::vector<Point_t> Pts;
	Point_t p1 { (float)int(vPt.x), (float)int(vPt.y), (float)int(vPt.z) };
	Pts.push_back(p1);
	Pts.push_back({ p1.x + 1, p1.y, p1.z });
	Pts.push_back({ p1.x, p1.y + 1, p1.z });
	Pts.push_back({ p1.x, p1.y, p1.z + 1 });
	Pts.push_back({ p1.x + 1, p1.y + 1, p1.z });
	Pts.push_back({ p1.x + 1, p1.y, p1.z + 1 });
	Pts.push_back({ p1.x, p1.y + 1, p1.z + 1 });
	Pts.push_back({ p1.x + 1, p1.y + 1, p1.z + 1 });

	float MinDist2 = 1.0f;
	Point_t Candi;
	for (const auto& e : Pts) {
		float Dist2 = (vPt.x - e.x) * (vPt.x - e.x) + (vPt.y - e.y) * (vPt.y - e.y) + (vPt.z - e.z) * (vPt.z - e.z);
		if (MinDist2 > Dist2) {
			MinDist2 = Dist2;
			Candi = e;
		}
	}
	
	return Candi;
}
