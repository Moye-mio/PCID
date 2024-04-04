#pragma once

class ImageUtil {
public:
	static float calcSSIM(const cv::Mat& a, const cv::Mat& b);
	static bool isEmptyInNeighbor(const cv::Mat& m, uint x, uint y);
	static bool isHoleBoundary(const cv::Mat& m, uint x, uint y, int d = 1);
	static bool isIndexValid(const cv::Mat& m, int x, int y);
	static bool isImageValid(const cv::Mat& m);
	static bool isImageInpaintingInputValid(const cv::Mat& vSrc, const cv::Mat& vMask);
	static bool saveToLocal(const cv::Mat& vSrc, const std::string& vPath, bool vIsNormalize);

private:

};