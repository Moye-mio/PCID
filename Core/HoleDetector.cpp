#include "pch.h"
#include "HoleDetector.h"
#include "PointCloudUtil.h"
#include "AABB.h"
#include "OutlierDetector.h"
#include <pcl/features/boundary.h>

using namespace core;

bool CHoleDetector::run(const PC_t::Ptr vCloud, float vRadius, float vThreshold) {
	_EARLY_RETURN(!isPointCloudValid(vCloud), "hole detector: point cloud is invalid.", false);

	pcl::PointCloud<pcl::Boundary> pBoundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> Est;
	pcl::PointCloud<pcl::Normal>::Ptr pNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto extractPositions = ([=](const PC_t::Ptr& vCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& voCloud) {
		for (const auto& p : *vCloud) {
			voCloud->emplace_back(pcl::PointXYZ(p.x, p.y, p.z));
		}
		}
	);

	auto extractNormals = ([=](const PC_t::Ptr& vCloud, pcl::PointCloud<pcl::Normal>::Ptr& voNormals) {
		for (const auto& p : *vCloud) {
			voNormals->emplace_back(pcl::Normal(p.normal_x, p.normal_y, p.normal_z));
		}
		}
	);

	extractPositions(vCloud, pCloud);
	extractNormals(vCloud, pNormals);

	Est.setInputCloud(pCloud);
	Est.setInputNormals(pNormals);
	Est.setRadiusSearch(vRadius);
	Est.setAngleThreshold(vThreshold);
	Est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	Est.compute(pBoundaries);

	for (int i = 0; i < vCloud->size(); i++) {
		if (pBoundaries[i].boundary_point > 0) {
			m_BounIndices.push_back(i);
		}
	}

	__removeBorder(vCloud);
	__removeOutlier(vCloud, vRadius);
	__removeFinal(vCloud);

	std::cout << "Boundary points number: " << m_BounIndices.size() << std::endl;

	return true;
}

void CHoleDetector::__removeBorder(const PC_t::Ptr vCloud) {
	SAABB Box = PointCloudUtil::calcAABB(vCloud);

	float k = 0.05f;
	float ThresX = (Box.maxx - Box.minx) * k;
	float ThresY = (Box.maxy - Box.miny) * k;

	std::vector<int> Removed;
	for (auto i : m_BounIndices) {
		const auto& p = vCloud->at(i);
		if (std::fabsf(p.x - Box.minx) < ThresX || std::fabsf(p.x - Box.maxx) < ThresX || std::fabsf(p.y - Box.miny) < ThresY || std::fabsf(p.y - Box.maxy) < ThresY) {
			continue;
		}
		else {
			Removed.push_back(i);
		}
	}

	std::cout << "Before remove point size: " << m_BounIndices.size() << ", after remove point size: " << Removed.size() << std::endl;

	m_BounIndices = Removed;
}

void CHoleDetector::__removeOutlier(const PC_t::Ptr vCloud, float vRadius) {
	std::vector<int> Inliers;

	auto getCloudFromIndices = ([=](const PC_t::Ptr& vCloud, PC_t::Ptr& voTarget) {
		for (auto i : m_BounIndices) {
			voTarget->emplace_back(vCloud->at(i));
		}
		}
	);

	PC_t::Ptr pInliers(new PC_t);
	getCloudFromIndices(vCloud, pInliers);
	COutlierDetecter Detecter;
	Detecter.run(pInliers, 10, vRadius);
	Detecter.dumpInliers(Inliers);

	for (int i = 0; i < Inliers.size(); i++) {
		Inliers[i] = m_BounIndices[Inliers[i]];
	}

	m_BounIndices = Inliers;
}

void core::CHoleDetector::__removeFinal(const PC_t::Ptr vCloud) {
	PC_t::Ptr pBound(new PC_t);
	for (auto i : m_BounIndices) {
		pBound->emplace_back(vCloud->at(i));
	}

	pcl::search::KdTree<Point_t>::Ptr pTree(new pcl::search::KdTree<Point_t>);
	pTree->setInputCloud(pBound);
	int k = 10;

	std::vector<float> TotalDists;
	float AverageDist = 0.0f;
	for (size_t i = 0; i < pBound->size(); i++) {
		std::vector<int> Indices;
		std::vector<float> Dists;

		pTree->nearestKSearch(pBound->at(i), k + 1, Indices, Dists);	/* Indices[0] is point itself. */

		float Dist = 0.0f;
		for (auto d : Dists) {
			Dist += d;
		}
		Dist /= k;
		AverageDist += Dist;
		TotalDists.push_back(Dist);
	}

	AverageDist /= pBound->size();

	std::vector<int> Copy = m_BounIndices;
	m_BounIndices.clear();
	for (int i = 0; i < pBound->size(); i++) {
		if (TotalDists[i] < AverageDist * 1.2) {
			m_BounIndices.push_back(Copy[i]);
		}
	}
}

