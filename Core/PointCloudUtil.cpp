#include "pch.h"
#include "PointCloudUtil.h"
#include "AABB.h"

using namespace core;

SAABB PointCloudUtil::calcAABB(const PC_t::Ptr vCloud) {
	SAABB Box { FLT_MAX, FLT_MAX , FLT_MAX , -FLT_MAX, -FLT_MAX , -FLT_MAX };
	_EARLY_RETURN(isPointCloudValid(vCloud) == false, "point cloud util error: input cloud is invalid.", Box);
	
	for (const auto& p : *vCloud) {
		Box.minx = std::fminf(Box.minx, p.x);
		Box.miny = std::fminf(Box.miny, p.y);
		Box.minz = std::fminf(Box.minz, p.z);
		Box.maxx = std::fmaxf(Box.maxx, p.x);
		Box.maxy = std::fmaxf(Box.maxy, p.y);
		Box.maxz = std::fmaxf(Box.maxz, p.z);
	}

	_EARLY_RETURN(Box.isValid() == false, "point cloud util error: aabb calculate bug.", Box);

	return Box;
}

bool PointCloudUtil::calcNormal(const PC_t::Ptr vioCloud, std::uint32_t k) {
	PT_t::Ptr pPts(new PT_t);
	__extractXYZpt(vioCloud, pPts);

	pcl::NormalEstimation<P_t, pcl::Normal> Ne;
	pcl::search::KdTree<P_t>::Ptr pTree(new pcl::search::KdTree<P_t>());
	pcl::PointCloud<pcl::Normal>::Ptr pNormal(new pcl::PointCloud<pcl::Normal>);

	Ne.setInputCloud(pPts);
	Ne.setSearchMethod(pTree);
	Ne.setKSearch(k);
	Ne.compute(*pNormal);

	_EARLY_RETURN(pNormal->size() != vioCloud->size(), "point cloud util error: Normal estimator error. ", false);

	for (int i = 0; i < vioCloud->size(); i++) {
		vioCloud->at(i).normal_x = pNormal->at(i).normal_x;
		vioCloud->at(i).normal_y = pNormal->at(i).normal_y;
		vioCloud->at(i).normal_z = pNormal->at(i).normal_z;
	}

	return true;
}

void PointCloudUtil::__extractXYZpt(const PC_t::Ptr& vCloud, const PT_t::Ptr& voPts) {
	if (voPts->size()) {
		voPts->clear();
	}

	for (const auto& p : *vCloud) {
		voPts->emplace_back(P_t(p.x, p.y, p.z));
	}
}
