#pragma once


class Task {

public:

	static cv::Mat generateMask(const std::string& vHolePath);
	static cv::Mat generateMask(const std::string& vHolePath, const std::string& vGTPath);
	static cv::Mat inpaintImage(const cv::Mat& vSrc, const cv::Mat& vMask, const cv::Mat& vGT);
	static cv::Mat reconstruct(const cv::Mat& vGT, const cv::Mat& vMask);
	static cv::Mat mergeImage(const cv::Mat& a, const cv::Mat& b, const cv::Mat& vMask);
};