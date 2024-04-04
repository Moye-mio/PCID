#include "header.h"

int main() {

	const std::string ImagePath = "Images/gt.png";
	const std::string MaskPath = "Images/mask.png";
	cv::Mat Input = cv::imread(ImagePath, cv::IMREAD_GRAYSCALE);
	cv::Mat Mask = cv::imread(MaskPath, cv::IMREAD_GRAYSCALE);
	
	for (int i = 0; i < magic_enum::enum_count<alg::EInpaintMode>(); i++) {
		cv::Mat Result;
		alg::CImageInpainting Inpainter;
		Inpainter.run(Input, Mask, Result, static_cast<alg::EInpaintMode>(i));
		cv::imwrite("Images/res-" + std::to_string(i) + ".png", Result);
	}

	return 0;
}





