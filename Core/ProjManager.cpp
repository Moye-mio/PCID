#include "pch.h"
#include "ProjManager.h"
#include "PointCloudUtil.h"

using namespace core;

bool CProjManager::calcDirection(std::vector<SProj>& vioProj) {

	PC_t::Ptr pCloud(new PC_t);
	__castProj2PCLpt(vioProj, pCloud);

	bool r = PointCloudUtil::calcNormal(pCloud, 10);
	_EARLY_RETURN(!r, "proj manager error: calc normal fails.", false);

	for (int i = 0; i < pCloud->size(); i++) {
		if (__isSameDirection(vioProj[i], pCloud->at(i)) == false) {
			vioProj[i].dist *= -1;
		}
	}

    return true;
}

void CProjManager::__castProj2PCLpt(const std::vector<SProj>& vProj, const PC_t::Ptr& voCloud) {
	for (auto& e : vProj) {
		voCloud->emplace_back(Point_t(e.pos.x, e.pos.y, e.pos.z));
	}
}

bool CProjManager::__isSameDirection(const SProj& vProj, const Point_t& vPt) {
	Eigen::Vector3f Pd { vProj.projDirection.x, vProj.projDirection.y, vProj.projDirection.z };
	Eigen::Vector3f Nd { vPt.normal_x, vPt.normal_y, vPt.normal_z };

	if (Pd.dot(Nd) > 0) {
		return true;
	}
	else {
		return false;
	}
}
