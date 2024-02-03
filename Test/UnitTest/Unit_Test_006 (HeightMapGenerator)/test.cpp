#include "pch.h"

template <typename T>
using ptr = std::shared_ptr<T>;

TEST(HMGenerator, SimpleCloud) {
	PC_t::Ptr pCloud(new PC_t);
	pCloud->emplace_back(Point_t(0.0f, 0, 1));
	pCloud->emplace_back(Point_t(0.0f, 2, 2));
	pCloud->emplace_back(Point_t(2.0f, 0, 3));
	pCloud->emplace_back(Point_t(2.0f, 2, 4));

	core::CHeightMapGenerator HMGenerator;
	auto pHeight = HMGenerator.generate(pCloud, 1, 1);
	EXPECT_TRUE(pHeight->isValid() && pHeight->isNoEmpty());
	EXPECT_TRUE(pHeight->getMax() == 4.0f && MathUtil::isEqual(pHeight->getMaxId(), std::pair<std::uint32_t, std::uint32_t>{0, 0}));

	pHeight = HMGenerator.generate(pCloud, 2, 2);
	EXPECT_TRUE(pHeight->isValid() && pHeight->isNoEmpty());
	EXPECT_TRUE(pHeight->getMax() == 4.0f && MathUtil::isEqual(pHeight->getMaxId(), std::pair<std::uint32_t, std::uint32_t>{1, 1}));

	pHeight = HMGenerator.generate(pCloud, 3, 3);
	EXPECT_TRUE(pHeight->isValid() && pHeight->getEmptyCount() == 5);
	EXPECT_TRUE(pHeight->isEmpty(0, 1) && pHeight->isEmpty(1, 0) && pHeight->isEmpty(1, 1) && pHeight->isEmpty(1, 2) && pHeight->isEmpty(2, 1));
	EXPECT_TRUE(MathUtil::isEqual(pHeight->getValue(0, 0), 1.0f) && MathUtil::isEqual(pHeight->getValue(0, 2), 2.0f) && MathUtil::isEqual(pHeight->getValue(2, 0), 3.0f));
	EXPECT_TRUE(pHeight->getMax() == 4.0f && MathUtil::isEqual(pHeight->getMaxId(), std::pair<std::uint32_t, std::uint32_t>{2, 2}));
}

TEST(HMGenerator, ComplexCloud) {
	core::ptr<core::CHeightMap> pGT(new core::CHeightMap(100, 100));

	PC_t::Ptr pCloud(new PC_t);
	for (int i = 0; i < 100; i++) {
		for (int k = 0; k < 100; k++) {
			float maxz = -FLT_MAX;
			for (int m = 0; m < 10; m++) {
				float x = MathUtil::geneRandomReal(0, 1);
				float y = MathUtil::geneRandomReal(0, 1);
				float z = MathUtil::geneRandomReal(0, 255);
				pCloud->emplace_back(Point_t(i + x, k + y, z));
				maxz = std::fmaxf(maxz, z);
			}
			pGT->setValue(i, k, maxz);
		}
	}
	pCloud->emplace_back(Point_t(0, 0, 255.0f));
	pCloud->emplace_back(Point_t(100, 100, 255.0f));
	pGT->setValue(0, 0, 255.0f);
	pGT->setValue(99, 99, 255.0f);

	core::CHeightMapGenerator HMGenerator;
	auto pHeight = HMGenerator.generate(pCloud, 100, 100);
	EXPECT_TRUE(pHeight->isValid() && pHeight->isNoEmpty());
	EXPECT_TRUE(pHeight->getMax() == 255.0f);

	for (int i = 0; i < 100; i++) {
		for (int k = 0; k < 100; k++) {
			EXPECT_TRUE(MathUtil::isEqual(pHeight->getValue(i, k), pGT->getValue(i, k)));
			if (MathUtil::isEqual(pHeight->getValue(i, k), pGT->getValue(i, k)) == false) {
				std::cout << "not equal: [" << i << ", " << k << "]: " << pHeight->getValue(i, k) << ", " << pGT->getValue(i, k) << std::endl;		/* sometimes bug because of floating-point precision  */
			}
		}
	}
}

TEST(HMGenerator, ComplexCloud2) {
	core::ptr<core::CHeightMap> pGT(new core::CHeightMap(100, 100));

	PC_t::Ptr pCloud(new PC_t);
	for (int i = 0; i < 100; i++) {
		for (int k = 0; k < 100; k++) {
			float maxz = -FLT_MAX;
			float minz = FLT_MAX;
			int posi = 0;
			int nega = 0;
			for (int m = 0; m < 11; m++) {
				float x = MathUtil::geneRandomReal(0, 1);
				float y = MathUtil::geneRandomReal(0, 1);
				float z = MathUtil::geneRandomReal(-255, 255);
				pCloud->emplace_back(Point_t(i + x, k + y, z));
				maxz = std::fmaxf(maxz, z);
				minz = std::fminf(minz, z);

				if (z > 0)
					posi++;
				else if (z < 0)
					nega++;
			}
			if (posi >= nega)
				pGT->setValue(i, k, maxz);
			else 
				pGT->setValue(i, k, minz);
		}
	}
	pCloud->emplace_back(Point_t(0, 0, 0.0f));
	pCloud->emplace_back(Point_t(100, 100, 0.0f));

	core::CHeightMapGenerator HMGenerator;
	auto pHeight = HMGenerator.generate(pCloud, 100, 100);
	EXPECT_TRUE(pHeight->isValid() && pHeight->isNoEmpty());

	for (int i = 0; i < 100; i++) {
		for (int k = 0; k < 100; k++) {
			EXPECT_TRUE(MathUtil::isEqual(pHeight->getValue(i, k), pGT->getValue(i, k)));
			if (MathUtil::isEqual(pHeight->getValue(i, k), pGT->getValue(i, k)) == false) {
				std::cout << "not equal: [" << i << ", " << k << "]: " << pHeight->getValue(i, k) << ", " << pGT->getValue(i, k) << std::endl;		/* sometimes bug because of floating-point precision  */
			}
		}
	}
}

TEST(HMGenerator, LoadCloud) {
	const std::string Path = TESTMODEL_DIR + std::string("BigTerrain.ply");
	io::CPCLoader Loader;
	PC_t::Ptr pCloud = Loader.loadDataFromFile(Path);
	_ASSERTE(isPointCloudValid(pCloud));

	core::CHeightMapGenerator HMGenerator;
	auto pHeight = HMGenerator.generate(pCloud, 70, 70);
	EXPECT_TRUE(pHeight->isValid() && pHeight->isNoEmpty());

	cv::Mat Image = core::CMapWrapper::castMap2CVMat<float>(pHeight);
	cv::Mat SaveImage(Image.size(), CV_8UC1);
	float Scale = 255.0f / (pHeight->getMax() - pHeight->getMin());
	Image.convertTo(SaveImage, CV_8UC1, Scale, -pHeight->getMin() * Scale);
	cv::imwrite("Images/Test2.png", SaveImage);
}
