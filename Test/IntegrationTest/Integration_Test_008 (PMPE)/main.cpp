#include "header.h"

template <typename T>
using ptr = std::shared_ptr<T>;
using hmptr = ptr<core::CHeightMap>;
using mmptr = ptr<core::CMaskMap>;
using gmptr = ptr<core::CGradientMap>;

int main() {
	cv::Mat Raw = cv::imread("Images/1.png", cv::IMREAD_GRAYSCALE);
	cv::Mat Mask = cv::imread("Images/1-mask.png", cv::IMREAD_GRAYSCALE);
	cv::Mat Gt(Raw.size(), CV_32FC1);
	Raw.convertTo(Gt, CV_32FC1);
	cv::Mat Res;

	alg::CPoissonImageInpainting Inpainter;
	Inpainter.run(Gt, Mask, Res, alg::EPoissonGradient::MIX);

	cv::imwrite("Images/1-pmmix.png", Res);

	return 0;
}
