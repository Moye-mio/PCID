#include "pch.h"

TEST(Gradient, Basic_Calculation) {
	cv::Mat Image = cv::imread("Images/1.png", 0);
	cv::Mat h;
	Image.convertTo(h, CV_32FC1);
	std::shared_ptr<core::CHeightMap> pHeight = std::get<0>(core::CMapWrapper::castCVMat2Map(h));
	std::shared_ptr<core::CGradientMap> pGradient = core::MapUtil::geneGradient(pHeight);

	EXPECT_TRUE(pGradient->getValue(26, 0).x, -30);
	EXPECT_TRUE(pGradient->getValue(42, 0).x, 30);
	EXPECT_TRUE(pGradient->getValue(54, 0).x, -83.5);
	EXPECT_TRUE(pGradient->getValue(72, 0).x, 83.5);
	EXPECT_TRUE(pGradient->getValue(85, 0).x, -127.5);

	core::CMapWrapper::saveGMapToLocal(pGradient, "Images/g");
}
