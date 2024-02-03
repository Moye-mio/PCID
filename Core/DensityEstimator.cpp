#include "pch.h"
#include "DensityEstimator.h"

using namespace core;

float CDensityEstimator::run(const PC_t::Ptr vCloud, std::uint32_t k) {
	_EARLY_RETURN(!isPointCloudValid(vCloud), "density estimator: cloud is not valid.", 0.0f);
	
	pcl::search::KdTree<Point_t>::Ptr pTree(new pcl::search::KdTree<Point_t>);
	pTree->setInputCloud(vCloud);
	float GlobalDensity = 0.0f;

	for (size_t i = 0; i < vCloud->size(); i++) {
		std::vector<int> Indices;
		std::vector<float> Dists;
		
		pTree->nearestKSearch(vCloud->at(i), k + 1, Indices, Dists);	/* Indices[0] is point itself. */

		float Density = 0.0f;
		for (auto e : Dists) {
			Density += e;
		}
		GlobalDensity += Density / k;
	}

	GlobalDensity /= vCloud->size();

	std::cout << "Density estimator complete: " << GlobalDensity << std::endl;
	return GlobalDensity;
}
