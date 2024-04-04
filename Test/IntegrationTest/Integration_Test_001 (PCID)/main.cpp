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

/* BOOST */
#include <boost/format.hpp>

/* CV */
#include <opencv2/opencv.hpp>

/* COMMON */
#include "Common.h"

/* IO */
#include "PCLoader.h"

/* CORE */
#include "MapUtil.h"
#include "HeightMap.h"
#include "GradientMap.h"
#include "MaskMap.h"
#include "HeightMapGenerator.h"
#include "MapWrapper.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"
#include "EigenUtil.h"
#include "DensityEstimator.h"
#include "SurfaceUtil.h"
#include "Proj.h"
#include "PointCloudUtil.h"
#include "AABB.h"
#include "DuplicateRemover.h"

/* ALG */
#include "ImageInpainting.h"
#include "ImageUtil.h"

/* DGI */
#include "PCID.h"

struct SFitPara {
	int d;
	int r;
	int it;
};

struct SRes {
	int work;
	int reco;
};


int main() {
	SFitPara Para { 3, 5, 10 };
	SRes Res { 100, 300 };
	PC_t::Ptr pInput(new PC_t), pSub(new PC_t), pOutput(new PC_t);

	std::vector<std::string> FileIndices;
	FileIndices.push_back("1");
	//FileIndices.push_back("2");
	//FileIndices.push_back("3");
	//FileIndices.push_back("6");

	for (const std::string& Id : FileIndices) {
		if (Id == "6") {
			Res = { 100, 300 };
		}

		const std::string InputLoadPath = MAINEXPERIMENT_DIR + std::string("Hole/" + Id + ".ply");
		const std::string SubLoadPath = MAINEXPERIMENT_DIR + std::string("sub/" + Id + ".ply");
		const std::string SavePath = "Result/" + Id + ".ply";

		io::CPCLoader Loader;
		pInput = Loader.loadDataFromFile(InputLoadPath);
		pSub = Loader.loadDataFromFile(SubLoadPath);

		CPCID PCID;
		PCID.setResolution(Res.work, Res.reco);
		PCID.setFitPara(Para.d, Para.r, Para.it);
		bool r = PCID.run(pInput, pSub, pOutput);

		if (r == false) {
			log("PCID error.");
		}
		else {
			log("PCID succeeds.");
		}

		*pOutput += *pInput;

		pcl::io::savePLYFileBinary(SavePath, *pOutput);
	}

	return 0;
}

