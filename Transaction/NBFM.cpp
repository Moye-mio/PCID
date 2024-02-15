#include "pch.h"
#include "NBFM.h"
#include "Voxelizer.h"
#include "CubeUtil.h"
#include "CubeSplitter.h"
#include "HoleDetector.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>

bool CNBFM::run(const PC_t::Ptr vCloud) {
	_EARLY_RETURN(!isPointCloudValid(vCloud), "NBFM error: input is invalid.", false);

	int MaxIteration = 5;

	
	
	PC_t::Ptr pWork = vCloud;

	for (int it = 0; it < MaxIteration; it++) {
		/*core::CVoxelizer Voxelizer;
		PC_t::Ptr pVoxels = Voxelizer.run(pWork);
		core::SAABB Box1 = core::PointCloudUtil::calcAABB(vCloud);
		core::SAABB Box2 = core::PointCloudUtil::calcAABB(pVoxels);
		pWork = pVoxels;*/

		//float Scale = (Box1.maxx - Box1.minx) / (Box2.maxx - Box2.minx);
		//_EARLY_RETURN(!isPointCloudValid(pVoxels) || Scale <= 0.0f, "NBFM error: voxelizer fails", false);

		float Scale = 1.0f;

		bool r = core::PointCloudUtil::calcNormal(pWork, 10);
		_EARLY_RETURN(!r, "nbfm error: voxels calc normal fails", false);

		std::vector<core::SCube> Cubes;
		core::SAABB Box = core::PointCloudUtil::calcAABB(pWork);
		core::CCubeSplitter Splitter;
		float HalfCubeLength = (Box.maxx - Box.minx) / 5 / 2;
		r = Splitter.run(pWork, vec3f { Box.minx - (it % 2) * HalfCubeLength, Box.miny - (it % 2) * HalfCubeLength, Box.minz - (it % 2) * HalfCubeLength }, HalfCubeLength * 2);
		Splitter.dumpCubes(Cubes);
		_EARLY_RETURN(!r, "nbfm error: splitter fails", false);

		std::vector<int> Indices;
		core::CHoleDetector Detector;
		r = Detector.run(pWork, (Box.maxx - Box.minx) / 30, std::numbers::pi / 2.0f);
		Detector.dumpBoundaryIndices(Indices);
		_EARLY_RETURN(!r, "nbfm error: hole detector fails", false);

		__maskCubesWithHole(Cubes, Indices);

		PC_t::Ptr pNew(new PC_t);
		for (int i = 0; i < Cubes.size(); i++) {
			if (Cubes[i].ishole) {
				Cubes[i].sourceid = __calcNearestCube(pWork, Cubes, i);

				Eigen::Matrix4f Transform;
				PC_t::Ptr pResult = __calcTransfromBasedOnICP(pWork, Cubes[i], Cubes[Cubes[i].sourceid]);
				*pNew += *pResult;
			}
			else {
				for (auto id : Cubes[i].indices) {
					pNew->emplace_back(pWork->at(id));
				}
			}
		}

		for (auto& p : *pNew) {
			p.x *= Scale;
			p.y *= Scale;
			p.z *= Scale;
		}

		__mergeNewWorkPointCloud(pWork, pNew);

		pcl::io::savePLYFileBinary("Result/new-" + std::to_string(it) + ".ply", *pNew);

		pWork = pNew;
	}

	std::cout << "nbfm complete." << std::endl;

	return true;
}

void CNBFM::__maskCubesWithHole(std::vector<core::SCube>& vioCubes, const pcl::Indices& vIndices) {
	int HoleCubeCount = 0;
	
	for (auto& e : vioCubes) {
		for (auto i : e.indices) {
			for (auto ii : vIndices) {
				if (i == ii) {
					e.ishole = true;
					HoleCubeCount++;
					break;
				}
			}

			if (e.ishole == true) {
				break;
			}
		}
	}
	
	std::cout << "nbfm: " << vIndices.size() << " hole boundary points in " << HoleCubeCount << " cubes." << std::endl;
}

int CNBFM::__calcNearestCube(const PC_t::Ptr vCloud, const std::vector<core::SCube>& vCubes, int vTarget) {
	float MaxSimi = 0.0f;
	int SourceId = 0;

	for (int i = 0; i < vCubes.size(); i++) {
		if (i == vTarget || (vCubes[i].indices.size() < vCubes[vTarget].indices.size() / 2) && vCubes[vTarget].indices.size() > vCubes[i].indices.size() || vCubes[i].ishole) {
			continue;
		}

		float Simi = core::CubeUtil::calcCubeSimilarity(vCloud, vCubes[vTarget], vCubes[i]);
		if (Simi > MaxSimi) {
			SourceId = i;
		}
	}

	return SourceId;
}

void CNBFM::__calcTransform(const PC_t::Ptr vCloud, const core::SCube& t, const core::SCube& s, Eigen::Matrix3f& voRotation, vec3f& voTranslation) {
	float Thres = 0.5f;
	std::vector<vec3f> CandiInfos;

	for (auto tpid : t.indices) {
		const auto& p1 = vCloud->at(tpid);
		pcl::Indices Candidates;
		for (auto spid : s.indices) {
			const auto& p2 = vCloud->at(spid);
			if ((p1.normal_x * p2.normal_x + p1.normal_y * p2.normal_y + p1.normal_z * p2.normal_z) > Thres) {
				Candidates.push_back(spid);
			}
		}

		if (Candidates.empty()) {
			continue;
		}

		float MinDist = FLT_MAX;
		int BestId = 0;
		for (auto id : Candidates) {
			const auto& p = vCloud->at(id);
			float d = (p.x - p1.x) * (p.x - p1.x) + (p.y - p1.y) * (p.y - p1.y) + (p.z - p1.z) * (p.z - p1.z);
			if (MinDist < d) {
				MinDist = d;
				BestId = id;
			}
		}

		CandiInfos.emplace_back(vec3f {MinDist, (float)tpid, (float)BestId});
	}



}

PC_t::Ptr CNBFM::__calcTransfromBasedOnICP(const PC_t::Ptr vCloud, const core::SCube& t, const core::SCube& s) {
	PC_t::Ptr pTarget = core::CubeUtil::extractPtsFromCube(vCloud, t);
	PC_t::Ptr pSource = core::CubeUtil::extractPtsFromCube(vCloud, s);
	PC_t::Ptr pResult(new PC_t);

	pcl::IterativeClosestPoint<Point_t, Point_t> ICP;
	ICP.setMaximumIterations(50);
	ICP.setInputSource(pSource);
	ICP.setInputTarget(pTarget);
	ICP.align(*pResult);

	std::cout << "NBFM: align complete" << std::endl;
	
	return pResult;
}

void CNBFM::__mergeNewWorkPointCloud(const PC_t::Ptr vInput, PC_t::Ptr& vioWork) {
	core::CDuplicateRemover Remover;
	bool r = Remover.run(vInput, vioWork, 10);
	_EARLY_RETURN(!r, "nbfm error: remove excess points fails.", );

	*vioWork += *vInput;
}

