#include "pch.h"

TEST(Density, NT_SimpleCloud) {
	PC_t::Ptr pCloud(new PC_t);
	pCloud->emplace_back(Point_t(0.0f, 0, 0));
	pCloud->emplace_back(Point_t(1.0f, 0, 0));
	pCloud->emplace_back(Point_t(0.0f, 1, 0));
	pCloud->emplace_back(Point_t(1.0f, 1, 0));

	core::CDensityEstimator de;
	EXPECT_EQ(de.run(pCloud, 2), 1.0f);
}


