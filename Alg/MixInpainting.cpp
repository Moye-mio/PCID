#include "pch.h"
#include "MixInpainting.h"
#include "PMInterface.h"
#include "CVInpainting.h"

using namespace alg;

CMixInpainting::CMixInpainting() 
	: m_CVType(ECVInpaintingType::NS)
	, m_Dist(2)
{}

void CMixInpainting::setCVInpaintingType(ECVInpaintingType vType) {
	m_CVType = vType;
}

void CMixInpainting::setDistThres(uint k) {
	m_Dist = k;
}

bool CMixInpainting::run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes) {
	_EARLY_RETURN(ImageUtil::isImageInpaintingInputValid(vSrc, vMask) == false, "", false);

	cv::Mat PMRes = PM::run(vSrc, vMask, 11);
	_EARLY_RETURN(ImageUtil::isImageValid(PMRes) == false, "Mix inpainting: pm error.", false);

	cv::Mat CVRes;
	CCVInpainting CVInpainter;
	CVInpainter.run(vSrc, vMask, CVRes, m_CVType);
	_EARLY_RETURN(ImageUtil::isImageValid(CVRes) == false, "Mix inpainting: CV error.", false);

	voRes = vSrc.clone();

	std::vector<cv::Mat> PMChannels, CVChannels, ResChannels;
	cv::split(PMRes, PMChannels);
	cv::split(CVRes, CVChannels);
	cv::split(voRes, ResChannels);

	for (int i = 0; i < vSrc.channels(); i++) {
		cv::Mat& PMCur = PMChannels.at(i);
		cv::Mat& CVCur = CVChannels.at(i);
		cv::Mat& ResCur = ResChannels.at(i);

		for (int i = 0; i < vMask.rows; i++) {
			for (int k = 0; k < vMask.cols; k++) {
				bool IsCV = ImageUtil::isHoleBoundary(vMask, i, k, m_Dist);
				vec2f Extreme = __isCVExtreme(PMCur, vMask);
				if (CVCur.type() == CV_32FC1) {
					IsCV = IsCV && (Extreme.x <= CVCur.at<float>(i, k)) && (CVCur.at<float>(i, k) <= Extreme.y);
				}
				else if (CVCur.type() == CV_8UC1) {
					IsCV = IsCV && (Extreme.x <= (float)CVCur.at<uchar>(i, k)) && ((float)CVCur.at<uchar>(i, k) <= Extreme.y);
				}

				if (IsCV) {
					if (CVCur.type() == CV_32FC1) {
						ResCur.at<float>(i, k) = CVCur.at<float>(i, k);
					}
					else if (CVCur.type() == CV_8UC1) {
						ResCur.at<uchar>(i, k) = CVCur.at<uchar>(i, k);
					}
				}
				else {
					if (CVCur.type() == CV_32FC1) {
						ResCur.at<float>(i, k) = PMCur.at<float>(i, k);
					}
					else if (CVCur.type() == CV_8UC1) {
						ResCur.at<uchar>(i, k) = PMCur.at<uchar>(i, k);
					}
				}
			}
		}

		ImageUtil::saveToLocal(PMCur, "Images/PM-" + std::to_string(i) + ".png", true);
		ImageUtil::saveToLocal(CVCur, "Images/CV-" + std::to_string(i) + ".png", true);
		ImageUtil::saveToLocal(ResCur, "Images/Res-" + std::to_string(i) + ".png", true);
	}

	cv::merge(ResChannels, voRes);
	_EARLY_RETURN(ImageUtil::isImageValid(voRes) == false, "Mix inpainting: res error.", false);

	return true;
}

vec2f CMixInpainting::__isCVExtreme(const cv::Mat& vSrc, const cv::Mat& vMask) {
	cv::Mat Work;
	vSrc.convertTo(Work, CV_32FC1);

	float MaxValue = -FLT_MAX, MinValue = FLT_MAX;
	for (int i = 0; i < Work.rows; i++) {
		for (int k = 0; k < Work.cols; k++) {
			if (vMask.at<uchar>(i, k) == 0) {
				MaxValue = std::fmaxf(MaxValue, Work.at<float>(i, k));
				MinValue = std::fminf(MinValue, Work.at<float>(i, k));
			}
		}
	}

	return vec2f { MinValue, MaxValue };
}
