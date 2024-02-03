#include "pch.h"

TEST(Surface, NT_LoadCloud) {
	const std::string CloudPath = TESTMODEL_DIR + std::string("SmallTerrain.ply");

	PC_t::Ptr pCloud(new PC_t);
	io::CPCLoader Loader;
	pCloud = Loader.loadDataFromFile(CloudPath);

	core::CNurbsFitting Fitting;
	Fitting.setSavePath("ControlPoints/pts.txt");
	EXPECT_TRUE(Fitting.run(pCloud, 3, 5, 1));
}


