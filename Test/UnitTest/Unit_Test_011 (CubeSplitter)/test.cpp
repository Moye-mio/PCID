#include "pch.h"
#include "CubeUtil.h"

TEST(CubeSplitter, NT_LoadCloud) {
	const std::string CloudPath = MAINEXPERIMENT_DIR + std::string("Hole/2.ply");
	PC_t::Ptr pCloud(new PC_t);
	io::CPCLoader Loader;
	pCloud = Loader.loadDataFromFile(CloudPath);

	core::SAABB Box = core::PointCloudUtil::calcAABB(pCloud);
	core::CCubeSplitter Splitter;
	bool r = Splitter.run(pCloud, vec3f { Box.minx, Box.miny, Box.minz }, 100);
	EXPECT_TRUE(r);

	std::vector<core::SCube> Cubes;
	Splitter.dumpCubes(Cubes);

	for (const auto& e : Cubes) {
		PC_t::Ptr pCube(new PC_t);
		for (auto i : e.indices) {
			pCube->emplace_back(pCloud->at(i));
		}
		pcl::io::savePLYFileBinary("Cubes/Cube-" + std::to_string(e.id.x) + "-" + std::to_string(e.id.y) + "-" + std::to_string(e.id.z) + ".ply", *pCube);
	}


	float Similarity = core::CubeUtil::calcCubeSimilarity(pCloud, Cubes[3], Cubes[2]);

}


