#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
#include <variant>
#include <optional>

/* PCL */
#include <pcl/features/normal_3d_omp.h>

/* COMMON */
#include "Common.h"

/* CORE */
#include "SimilarityEstimator.h"

/* IO */
#include "PCLoader.h"


int main() {
	std::string FileId[] = { "1","2","3","6" };
	for (auto e : FileId) {
		io::CPCLoader Loader;
		const std::string GtPath = MAINEXPERIMENT_DIR + std::string("GT/" + e +".ply");
		const std::string ResultPath = MAINEXPERIMENT_DIR + std::string("Result/" + e +"/nbfm.ply");
		PC_t::Ptr pGt = Loader.loadDataFromFile(GtPath);
		PC_t::Ptr pNBFM = Loader.loadDataFromFile(ResultPath);

		core::CSimilarityEstimator se;
		float gpsnr = se.compute(pGt, pNBFM, core::ESimilarityMode::GPSNR).value();
		float nshd = se.compute(pGt, pNBFM, core::ESimilarityMode::NSHD).value();

		std::cout << e << "\tgpsnr: " << gpsnr << ", nshd: " << nshd << std::endl;
	}

	return 0;
}


