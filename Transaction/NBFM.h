#pragma once

#include "Cube.h"

class CNBFM {

public:

	bool run(const PC_t::Ptr vCloud);


private:
	void __maskCubesWithHole(std::vector<core::SCube>& vioCubes, const pcl::Indices& vIndices);
	int __calcNearestCube(const PC_t::Ptr vCloud, const std::vector<core::SCube>& vCubes, int vTarget);
	void __calcTransform(const PC_t::Ptr vCloud, const core::SCube& vTCubes, const core::SCube& vSCubes, Eigen::Matrix3f& voRotation, vec3f& voTranslation);
	PC_t::Ptr __calcTransfromBasedOnICP(const PC_t::Ptr vCloud, const core::SCube& vTCubes, const core::SCube& vSCubes);
	void __mergeNewWorkPointCloud(const PC_t::Ptr vInput, PC_t::Ptr& vioWork);

};

