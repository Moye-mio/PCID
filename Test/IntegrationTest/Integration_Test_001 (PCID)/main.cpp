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


int main() {
	int WorkResolution = 200;
	int RecoResolution = 512;
	PC_t::Ptr pInput(new PC_t), pSub(new PC_t), pOutput(new PC_t);
	const std::string InputLoadPath = MAINEXPERIMENT_DIR + std::string("hole/2.ply");
	const std::string SubLoadPath = MAINEXPERIMENT_DIR + std::string("sub/2.ply");
	//const std::string InputLoadPath = LOWFREQUENCY_DIR + std::string("hole.ply");
	//const std::string SubLoadPath = LOWFREQUENCY_DIR + std::string("sub.ply");
	const std::string SavePath = "Result/output.ply";

	io::CPCLoader Loader;
	pInput = Loader.loadDataFromFile(InputLoadPath);
	pSub = Loader.loadDataFromFile(SubLoadPath);

	CPCID PCID;
	PCID.setResolution(WorkResolution, RecoResolution);
	bool r = PCID.run(pInput, pSub, pOutput);

	if (r == false) {
		log("PCID error.");
	}
	else {
		log("PCID succeeds.");
	}

	pcl::io::savePLYFileBinary(SavePath, *pOutput);

	return 0;
}

