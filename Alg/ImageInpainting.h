#pragma once

namespace alg {

enum EInpaintMode {
	CV_NS,
	CV_TEALA,
	PM,
};

class CImageInpainting {
public:
	CImageInpainting() = default;
	~CImageInpainting() = default;

	bool run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, EInpaintMode vMode);

private:
	void __inpaintByOpenCV(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, int vFlag);

};

}
