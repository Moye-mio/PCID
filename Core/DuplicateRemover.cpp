#include "pch.h"
#include "DuplicateRemover.h"
#include "DensityEstimator.h"

using namespace core;

bool CDuplicateRemover::run(const PC_t::Ptr vRefer, PC_t::Ptr& vioResult, std::uint32_t k) {
	_EARLY_RETURN(!isPointCloudValid(vRefer), "DuplicateRemover: cloud is not valid.", false);

	core::CDensityEstimator DE;
	float Density = DE.run(vRefer, k);
	_EARLY_RETURN(Density == 0.0f, "DuplicateRemover: density 0.", false);

	pcl::search::KdTree<Point_t>::Ptr pTree(new pcl::search::KdTree<Point_t>);
	pTree->setInputCloud(vRefer);

	PC_t::Ptr pNew(new PC_t);
	for (int i = 0; i < vioResult->size(); i++) {
		std::vector<int> Indices;
		std::vector<float> Dists;
		pTree->nearestKSearch(vioResult->at(i), k + 1, Indices, Dists);
		
		if (Dists[0] > Density) {
			pNew->emplace_back(vioResult->at(i));
		}
	}

	std::cout << "DuplicateRemover: remove " << vioResult->size() - pNew->size() << " points." << std::endl;
	vioResult = pNew;

	return true;
}
