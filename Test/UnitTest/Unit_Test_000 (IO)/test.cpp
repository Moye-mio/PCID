#include "pch.h"


TEST(IO, Loader) {
	const std::string Path = TESTMODEL_DIR + std::string("000.ply");

	PC_t::Ptr pCloud(new PC_t);
	io::CPCLoader Loader;
	pCloud = Loader.loadDataFromFile(Path);
	EXPECT_EQ(pCloud->size(), 16);
}


