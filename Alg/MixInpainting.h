#pragma once

namespace alg {

class CMixInpainting {

public:
	CMixInpainting();

	void setCVInpaintingType(ECVInpaintingType vType);
	void setDistThres(uint k);
	bool run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes);

private:
	vec2f __isCVExtreme(const cv::Mat& vSrc, const cv::Mat& vMask);

private:
	uint m_Dist;
	ECVInpaintingType m_CVType;
};

}
