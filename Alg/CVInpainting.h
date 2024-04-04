#pragma once

namespace alg {

class CCVInpainting {

public:
	CCVInpainting();

	bool setDistThres(float vDist);
	bool run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, ECVInpaintingType vMode);

private:
	float m_Dist;

};

}
