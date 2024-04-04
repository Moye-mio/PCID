#include "pch.h"
#include "CVInpainting.h"
#include "ImageUtil.h"

using namespace alg;

CCVInpainting::CCVInpainting() 
	: m_Dist(1.0f)
{}

bool CCVInpainting::setDistThres(float vDist) {
	_EARLY_RETURN(vDist <= 1.0f, "Dist set invalid.", false);
	m_Dist = vDist;
	return true;
}

bool CCVInpainting::run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, ECVInpaintingType vMode) {
	_EARLY_RETURN(ImageUtil::isImageInpaintingInputValid(vSrc, vMask) == false, "", false);

	voRes = vSrc.clone();

	std::vector<cv::Mat> SrcChannels, ResChannels;
	cv::split(vSrc, SrcChannels);
	cv::split(voRes, ResChannels);

	for (int i = 0; i < vSrc.channels(); i++) {
		cv::Mat& SrcCur = SrcChannels.at(i);
		cv::Mat& ResCur = ResChannels.at(i);

		switch (vMode) {
		case ECVInpaintingType::TELEA:
			cv::inpaint(SrcCur, vMask, ResCur, m_Dist, cv::INPAINT_TELEA);
			break;
		case ECVInpaintingType::NS:
			cv::inpaint(SrcCur, vMask, ResCur, m_Dist, cv::INPAINT_NS);
			break;
		default:
			break;
		}
	}

	cv::merge(ResChannels, voRes);

	return true;
}
