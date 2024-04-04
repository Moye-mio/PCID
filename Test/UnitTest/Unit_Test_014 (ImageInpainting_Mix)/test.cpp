#include "pch.h"

TEST(PM, LoadImage_100) {
	const std::string HoleImagePath = "Images/1.png";
	const std::string MaskImagePath = "Images/1-mask.png";

	cv::Mat Hole = cv::imread(HoleImagePath, CV_8UC1);
	cv::Mat Mask = cv::imread(MaskImagePath, CV_8UC1);
	cv::Mat mix;

	alg::CImageInpainting Inpainter;
	EXPECT_TRUE(Inpainter.run(Hole, Mask, mix, alg::EInpaintMode::Mix));

	cv::imwrite("Images/mix.png", mix);

	alg::CMixInpainting MixInpainter;
	EXPECT_TRUE(MixInpainter.run(Hole, Mask, mix));

	cv::imwrite("Images/mix-2.png", mix);
}
