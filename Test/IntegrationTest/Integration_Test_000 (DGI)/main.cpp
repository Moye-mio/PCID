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
#include "HeightMap2PCMapper.h"
#include "MapWrapper.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"
#include "EigenUtil.h"

/* ALG */
#include "ImageInpainting.h"
#include "ImageUtil.h"

/* DGI */
#include "DGI.h"


int main() {
	int Resolution = 256;
	PC_t::Ptr pInput(new PC_t), pOutput(new PC_t);
	const std::string LoadPath = MODEL_DIR + std::string("hole/1.ply");
	const std::string SavePath = "Result/output.ply";

	io::CPCLoader Loader;
	pInput = Loader.loadDataFromFile(LoadPath);

	CDGI DGI;
	DGI.setResolution(Resolution);
	bool r = DGI.run(pInput, pOutput);

	if (r == false) {
		log("DGI error.");
	}
	else {
		log("DGI succeeds.");
	}

	pcl::io::savePLYFileBinary(SavePath, *pOutput);

	return 0;
}

