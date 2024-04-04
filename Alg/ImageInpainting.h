#pragma once

#include "AlgCommon.h"

namespace alg {

class CImageInpainting {
public:
	CImageInpainting() = default;
	~CImageInpainting() = default;

	bool run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, EInpaintMode vMode);

private:
	void __inpaintByOpenCV(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, int vFlag);
	void __inpaintByMix(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes);

};

}
