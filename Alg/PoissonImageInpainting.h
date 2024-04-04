#pragma once

#include "AlgCommon.h"

namespace alg {

class CPoissonImageInpainting {

public:

	void setGT(const cv::Mat& vGT) { m_GT = vGT; }
	bool run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, EPoissonGradient vMode);

private:
	void __mergeGT(cv::Mat& vioGradientResult);

private:
	cv::Mat m_Work;
	cv::Mat m_GT;
};

}

