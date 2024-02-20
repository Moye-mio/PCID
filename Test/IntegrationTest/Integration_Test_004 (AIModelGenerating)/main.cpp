#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
#include <variant>

/* PCL */
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/random_sample.h>

/* COMMON */
#include "Common.h"

/* CORE */
#include "AABB.h"
#include "PointCloudUtil.h"
#include "Cube.h"
#include "CubeSplitter.h"
#include "CubeUtil.h"
#include "DuplicateRemover.h"

/* IO */
#include "PCLoader.h"

using P_t = pcl::PointXYZ;
using PT_t = pcl::PointCloud<P_t>;

int main() {
	{
		const std::string HolePath = MAINEXPERIMENT_DIR + std::string("hole/6.ply");
		const std::string Path = MAINEXPERIMENT_DIR + std::string("Result/6-res.ply");
		io::CPCLoader Loader;
		PC_t::Ptr pHole = Loader.loadDataFromFile(HolePath);
		PC_t::Ptr pInput = Loader.loadDataFromFile(Path);

		core::CDuplicateRemover Remover;
		Remover.run(pHole, pInput, 5);
		pcl::io::savePLYFileBinary(MAINEXPERIMENT_DIR + std::string("Result/6-csf.ply"), *pInput);
	}

	{
		const std::string Dir = "data/hyperCD/cull/";
		const std::string FileId[] = { "1.ply","2.ply","3.ply","6.ply" };
		const std::string HoleDir = MAINEXPERIMENT_DIR + std::string("hole/");
		const std::string AfterScaleSaveDir = "data/hyperCD/afterscale/";
		const std::string MergeSaveDir = "data/hyperCD/merge/";

		for (auto e : FileId) {
			io::CPCLoader Loader;
			PC_t::Ptr pInput = Loader.loadDataFromFile(Dir + e);
			PC_t::Ptr pHole = Loader.loadDataFromFile(HoleDir + e);

			core::PointCloudUtil::normalizeByReference(pInput, pHole);

			core::CDuplicateRemover Remover;
			bool r = Remover.run(pHole, pInput, 5);

			pcl::io::savePLYFileBinary(AfterScaleSaveDir + e, *pInput);

			*pInput += *pHole;
			pcl::io::savePLYFileBinary(MergeSaveDir + e, *pInput);
		}
	}


	{
		const std::string Dir = "data/casc/raw/";
		for (int i = 3; i < 20; i++) {
			const std::string FileId[] = { "_0_pred.ply","_1_pred.ply","_2_pred.ply","_3_pred.ply" };
			for (auto e : FileId) {
				io::CPCLoader Loader;
				PC_t::Ptr pFilled = Loader.loadDataFromFile(Dir + std::to_string(i * 10) + e);
				core::PointCloudUtil::normalize(pFilled);

				for (auto& p : *pFilled) {
					p.r = 0;
					p.g = 0;
					p.b = 0;
				}

				pcl::io::savePLYFileBinary("data/casc/normalize/" + std::to_string(i) + e, *pFilled);
			}
		}
	}

	{
		std::string Files[] = { "1", "2", "3", "6" };
		for (auto e : Files) {
			io::CPCLoader Loader;
			const std::string InputLoadPath = MAINEXPERIMENT_DIR + std::string("hole/" + e + ".ply");
			PC_t::Ptr pPartial = Loader.loadDataFromFile(InputLoadPath);
			core::PointCloudUtil::normalize(pPartial);

			PT_t::Ptr pSave(new PT_t);
			for (const auto& e : *pPartial) {
				pSave->emplace_back(P_t(e.x, e.y, e.z));
			}

			pcl::RandomSample<P_t> Sampling;
			Sampling.setInputCloud(pSave);
			Sampling.setSample(2048 - 512);
			Sampling.filter(*pSave);

			pcl::io::savePCDFileASCII("data/2048/partial-2048/" + e + ".pcd", *pSave);
		}
	}


	{
		std::string Files[] = { "1", "2", "3", "4", "5", "6", "7" };
		for (auto e : Files) {
			io::CPCLoader Loader;
			const std::string InputLoadPath = MAINEXPERIMENT_DIR + std::string("gt/" + e + ".ply");
			PC_t::Ptr pComplete = Loader.loadDataFromFile(InputLoadPath);	//"data/2048/complet-2048/" + e + ".ply"
			core::PointCloudUtil::normalize(pComplete);

			PT_t::Ptr pSave(new PT_t);
			for (const auto& e : *pComplete) {
				pSave->emplace_back(P_t(e.x, e.y, e.z));
			}
			
			pcl::RandomSample<P_t> Sampling;
			Sampling.setInputCloud(pSave);
			Sampling.setSample(2048);
			Sampling.filter(*pSave);

			pcl::io::savePCDFileASCII("data/2048/complet-2048/" + e + ".pcd", *pSave);
		}
	}

	{
		std::string Files[] = {"1", "2", "3", "6"};
		for (auto e : Files) {
			io::CPCLoader Loader;
			PC_t::Ptr pResult = Loader.loadDataFromFile("data/result/200/" + e + ".ply");
			PC_t::Ptr pRefer = Loader.loadDataFromFile(MAINEXPERIMENT_DIR + std::string("gt/" + e + ".ply"));

			core::PointCloudUtil::normalizeByReference(pResult, pRefer);
			
			pcl::io::savePLYFileBinary("data/result/200/" + e + "-scale.ply", *pResult);
		}
	}

	{
		io::CPCLoader Loader;
		PC_t::Ptr pComplete = Loader.loadDataFromFile("data/2048/test/complete/000/6.ply");

		PT_t::Ptr pSave(new PT_t);
		for (const auto&e : *pComplete) {
			pSave->emplace_back(P_t(e.x, e.y, e.z));
		}

		pcl::RandomSample<P_t> Sampling;
		Sampling.setInputCloud(pSave);
		Sampling.setSample(16384);
		Sampling.filter(*pSave);

		pcl::io::savePCDFileASCII("data/2048/test/complete/000/6.pcd", *pSave);
	}

	{
		std::string ModelId = std::to_string(6);
		io::CPCLoader Loader;
		PC_t::Ptr pComplete = Loader.loadDataFromFile(MAINEXPERIMENT_DIR + std::string("hole/" + ModelId + ".ply"));
		core::PointCloudUtil::normalize(pComplete);
		const std::string OutputLoadPath("data/hole/" + ModelId + ".pcd");
		
		pcl::io::savePCDFileASCII(OutputLoadPath, *pComplete);
	}


	for (int id = 1; id <= 7; id++) {
		std::string ModelId = std::to_string(id);
		const std::string InputLoadPath = MAINEXPERIMENT_DIR + std::string("gt/" + ModelId + ".ply");
		io::CPCLoader Loader;
		PC_t::Ptr pComplete = Loader.loadDataFromFile(InputLoadPath);
		int Res = 10;
		int DownSample = 2048;
		int ScanNum = Res * Res;

		core::PointCloudUtil::normalize(pComplete);
		core::SAABB Box = core::PointCloudUtil::calcAABB(pComplete);
		float HalfCubeLength = (Box.maxx - Box.minx) / Res / 2;
		std::vector<core::SCube> Cubes;
		core::CCubeSplitter Splitter;
		Splitter.run(pComplete, vec3f { Box.minx, Box.miny, Box.minz }, HalfCubeLength * 2);
		Splitter.dumpCubes(Cubes);

		for (int i = 0; i < ScanNum; i++) {
			PT_t::Ptr pPartial(new PT_t);
			for (int k = 0; k < Cubes.size(); k++) {
				if (i == k) {
					continue;
				}
				for (auto e : Cubes[k].indices) {
					auto p = pComplete->at(e);
					pPartial->emplace_back(P_t(p.x, p.y, p.z));
				}
			}

			pcl::RandomSample<P_t> Sampling;
			Sampling.setInputCloud(pPartial);
			Sampling.setSample(DownSample);
			Sampling.filter(*pPartial);

			const std::string OutputLoadPath("data/" + std::to_string(DownSample) + "/" + ModelId + "/partial/" + std::to_string(i) + ".pcd");
			pcl::io::savePCDFileASCII(OutputLoadPath, *pPartial);
		}

		PT_t::Ptr pCompleteSave(new PT_t);
		for (auto& p : *pComplete) {
			pCompleteSave->emplace_back(P_t(p.x, p.y, p.z));
		}

		pcl::RandomSample<P_t> Sampling;
		Sampling.setInputCloud(pCompleteSave);
		Sampling.setSample(DownSample * 8);
		Sampling.filter(*pCompleteSave);

		pcl::io::savePCDFileASCII("data/" + std::to_string(DownSample) + "/" + ModelId + "/complete/" + "complete.pcd", *pCompleteSave);
	}

	return 0;
}


