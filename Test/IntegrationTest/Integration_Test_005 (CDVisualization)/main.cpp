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

/* COMMON */
#include "Common.h"

/* CORE */
#include "AABB.h"
#include "PointCloudUtil.h"

/* IO */
#include "PCLoader.h"

using P_t = pcl::PointXYZ;
using PT_t = pcl::PointCloud<P_t>;

int main() {
	//std::string DataIds[] = { "3" };
	std::string DataIds[] = { "1","2","3","6" };
	//std::string Methods[] = { "csf" };
	std::string Methods[] = { "ours","csf","dgi","meshfix","nbfm" };
	for (auto e : DataIds) {
		io::CPCLoader Loader;
		const std::string GtPath = MAINEXPERIMENT_DIR + std::string("GT/" + e + ".ply");
		PC_t::Ptr pGt = Loader.loadDataFromFile(GtPath);
		core::PointCloudUtil::normalizeByFixScale(pGt);

		for (auto m : Methods) {
			const std::string InpaintedPath = MAINEXPERIMENT_DIR + std::string("Result/" + e + "/" + m + ".ply");
			PC_t::Ptr pInpainted = Loader.loadDataFromFile(InpaintedPath);

			core::PointCloudUtil::normalizeByFixScale(pInpainted);
			core::PointCloudUtil::calcChamferDistance(pInpainted, pGt, false);

			pcl::io::savePLYFileBinary("data/CDVisualization/" + e + "/" + m + ".ply", *pInpainted);
		}
		
	}

	



	return 0;
}


