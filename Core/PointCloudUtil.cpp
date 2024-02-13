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

float PointCloudUtil::calcChamferDistance(const PC_t::Ptr c1, const PC_t::Ptr c2) {
	int k = 1;
	float AverageCD1 = 0.0f;
	float AverageCD2 = 0.0f; 
	pcl::search::KdTree<Point_t>::Ptr pTree1(new pcl::search::KdTree<Point_t>);
	pTree1->setInputCloud(c1);

	for (size_t i = 0; i < c2->size(); i++) {
		std::vector<int> Indices;
		std::vector<float> Dists;

		pTree1->nearestKSearch(c2->at(i), k + 1, Indices, Dists);

		AverageCD1 += std::sqrtf(Dists[0]);
	}

	pcl::search::KdTree<Point_t>::Ptr pTree2(new pcl::search::KdTree<Point_t>);
	pTree2->setInputCloud(c2);

	for (size_t i = 0; i < c1->size(); i++) {
		std::vector<int> Indices;
		std::vector<float> Dists;

		pTree2->nearestKSearch(c1->at(i), k + 1, Indices, Dists);

		AverageCD2 += std::sqrtf(Dists[0]);
	}

	return AverageCD1 / c2->size() + AverageCD2 / c1->size();
}

void core::PointCloudUtil::calcChamferDistance(const PC_t::Ptr c1, const PC_t::Ptr c2, bool isBoth) {
	int k = 1;
	
	if (isBoth) {
		pcl::search::KdTree<Point_t>::Ptr pTree1(new pcl::search::KdTree<Point_t>);
		pTree1->setInputCloud(c1);

		for (size_t i = 0; i < c2->size(); i++) {
			std::vector<int> Indices;
			std::vector<float> Dists;

			pTree1->nearestKSearch(c2->at(i), k + 1, Indices, Dists);

			c2->at(i).normal_x = std::sqrtf(Dists[0]);
		}
	}

	pcl::search::KdTree<Point_t>::Ptr pTree2(new pcl::search::KdTree<Point_t>);
	pTree2->setInputCloud(c2);

	for (size_t i = 0; i < c1->size(); i++) {
		std::vector<int> Indices;
		std::vector<float> Dists;

		pTree2->nearestKSearch(c1->at(i), k + 1, Indices, Dists);

		c1->at(i).normal_x = std::sqrtf(Dists[0]);
		c1->at(i).normal_y = std::sqrtf(Dists[0]);
		c1->at(i).normal_z = Dists[0];
	}
}

bool PointCloudUtil::calcNormal(const PC_t::Ptr vioCloud, std::uint32_t k) {
	PT_t::Ptr pPts(new PT_t);
	extractXYZpt(vioCloud, pPts);

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

void PointCloudUtil::normalize(const PC_t::Ptr vioCloud) {
	SAABB Box = calcAABB(vioCloud);
	float Spanx = Box.maxx - Box.minx;
	float Spany = Box.maxy - Box.miny;
	float Spanz = Box.maxz - Box.minz;

	for (auto& p : *vioCloud){
		p.x -= Box.minx;
		p.y -= Box.miny;
		p.z -= Box.minz;
		p.x /= Spanx;
		p.y /= Spany;
		p.z /= Spanz;
	}
}

void core::PointCloudUtil::normalizeByFixScale(const PC_t::Ptr vioCloud, float vScale) {
	SAABB Box = calcAABB(vioCloud);
	float Spanx = Box.maxx - Box.minx;
	float Spany = Box.maxy - Box.miny;
	float Spanz = Box.maxz - Box.minz;
	if (MathUtil::isEqual(vScale, 1.0f)) {
		float SpanMax = std::fmaxf(Spanx, std::fmaxf(Spany, Spanz));

		for (auto& p : *vioCloud) {
			p.x -= Box.minx;
			p.y -= Box.miny;
			p.z -= Box.minz;
			p.x /= SpanMax;
			p.y /= SpanMax;
			p.z /= SpanMax;
		}
	}
	else {
		for (auto& p : *vioCloud) {
			p.x -= Box.minx;
			p.y -= Box.miny;
			p.z -= Box.minz;
			p.x /= vScale;
			p.y /= vScale;
			p.z /= vScale;
		}
	}
}

void PointCloudUtil::normalizeByReference(const PC_t::Ptr vioCloud, const PC_t::Ptr vRefer) {
	SAABB Box = calcAABB(vioCloud);
	SAABB ReferBox = calcAABB(vRefer);

	float Ratex = (ReferBox.maxx - ReferBox.minx) / (Box.maxx - Box.minx);
	float Ratey = (ReferBox.maxy - ReferBox.miny) / (Box.maxy - Box.miny);
	float Ratez = (ReferBox.maxz - ReferBox.minz) / (Box.maxz - Box.minz);

	for (auto& p : *vioCloud) {
		p.x -= Box.minx;
		p.y -= Box.miny;
		p.z -= Box.minz;
		p.x *= Ratex;
		p.y *= Ratey;
		p.z *= Ratez;
		p.x += ReferBox.minx;
		p.y += ReferBox.miny;
		p.z += ReferBox.minz;
	}
}

void PointCloudUtil::extractXYZpt(const PC_t::Ptr& vCloud, const PT_t::Ptr& voPts) {
	if (voPts->size()) {
		voPts->clear();
	}

	for (const auto& p : *vCloud) {
		voPts->emplace_back(P_t(p.x, p.y, p.z));
	}
}
