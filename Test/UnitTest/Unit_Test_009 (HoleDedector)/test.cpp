#include "pch.h"

TEST(HoleDetector, NT_LoadCloud) {
	const std::string CloudPath = MAINEXPERIMENT_DIR + std::string("Hole/2.ply");
	PC_t::Ptr pCloud(new PC_t);
	io::CPCLoader Loader;
	pCloud = Loader.loadDataFromFile(CloudPath);


	std::vector<int> Indices;
	core::CHoleDetector Detector;
	Detector.run(pCloud, 30, std::numbers::pi / 2.0f);
	Detector.dumpBoundaryIndices(Indices);

	PC_t::Ptr pBoundary(new PC_t);
	for (auto i : Indices) {
		auto& p = pCloud->at(i);
		p.r = 255;
		p.g = 0;
		p.b = 0;

		pBoundary->emplace_back(p);
	}

	pcl::io::savePLYFileBinary("2.ply", *pBoundary);
}


