#include "pch.h"
#include "ImageInpainting.h"
#include "PMInterface.h"

using namespace alg;

bool CImageInpainting::run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, EInpaintMode vMode) {
	_EARLY_RETURN(vSrc.size() != vMask.size(), "image inpainting error: src and mask is not the same size.", false);

	switch (vMode) {
	case CV_TEALA:
		__inpaintByOpenCV(vSrc, vMask, voRes, CV_TEALA);
		break;
	case CV_NS:
		__inpaintByOpenCV(vSrc, vMask, voRes, CV_NS);
		break;
	case PM:
		voRes = PM::run(vSrc, vMask);
		break;
	default:
		break;
	}

	return true;

}

void alg::CImageInpainting::__inpaintByOpenCV(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, int vFlag) {
	std::vector<cv::Mat> ChannelImages, ChannelInpainteds;
	cv::split(vSrc, ChannelImages);

	float Radius = 1.0f;

	for (int i = 0; i < vSrc.channels(); i++) {
		cv::Mat ChannelInpainted;
		cv::inpaint(ChannelImages[i], vMask, ChannelInpainted, Radius, vFlag);

		/* because cv inpaint makes some nan value in dst image, the subsequence step copies neighbor value to nan pixel.*/
		for (int k = 0; k < ChannelInpainted.rows; k++) {
			for (int m = 0; m < ChannelInpainted.cols; m++) {
				if (MathUtil::isNan(ChannelInpainted.at<float>(k, m))) {
					log(" ChannelInpainted cv mat value invalid: " + std::to_string(k) + ", " + std::to_string(m));
					if (k > 0 && !MathUtil::isNan(ChannelInpainted.at<float>(k - 1, m))) {
						ChannelInpainted.at<float>(k, m) = ChannelInpainted.at<float>(k - 1, m);
					}
					else if (m > 0 && !MathUtil::isNan(ChannelInpainted.at<float>(k, m - 1))) {
						ChannelInpainted.at<float>(k, m) = ChannelInpainted.at<float>(k, m - 1);
					}
					else if (k < ChannelInpainted.rows - 1 && !MathUtil::isNan(ChannelInpainted.at<float>(k + 1, m))) {
						ChannelInpainted.at<float>(k, m) = ChannelInpainted.at<float>(k + 1, m);
					}
					else if (m < ChannelInpainted.cols - 1 && !MathUtil::isNan(ChannelInpainted.at<float>(k, m + 1))) {
						ChannelInpainted.at<float>(k, m) = ChannelInpainted.at<float>(k, m + 1);
					}
				}
			}
		}
		ChannelInpainteds.push_back(ChannelInpainted);
	}

	cv::merge(ChannelInpainteds, voRes);
}

