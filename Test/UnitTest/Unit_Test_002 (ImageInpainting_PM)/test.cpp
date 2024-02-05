#include "pch.h"

TEST(PM, SmallImage_CV_32FC1) {
	const std::string TestDir = "Images/SmallImage/";

	cv::Mat Hole(cv::Size(11, 11), CV_32F);
	cv::Mat GT(cv::Size(11, 11), CV_32F);
	cv::Mat Mask(cv::Size(Hole.rows, Hole.cols), CV_8UC1);
	for (int i = 0; i < Hole.rows; i++)
		for (int k = 0; k < Hole.cols; k++) {
			if (i <= 5) {
				Hole.at<float>(i, k) = i * 50;
				GT.at<float>(i, k) = i * 50;
			}
			else {
				Hole.at<float>(i, k) = (10 - i) * 50;
				GT.at<float>(i, k) = (10 - i) * 50;
			}

			if (i >= 4 && i <= 6 && k >= 4 && k <= 6) {
				Mask.at<unsigned char>(i, k) = 255;
				Hole.at<float>(i, k) = 0;
			}
			else {
				Mask.at<unsigned char>(i, k) = 0;
			}
		}
	cv::imwrite(TestDir + "Hole.png", Hole);
	cv::imwrite(TestDir + "Mask.png", Mask);
	cv::Mat r = PM::run(Hole, Mask, 2);
	cv::imwrite(TestDir + "r.png", r);

	for (int i = 0; i < Hole.rows; i++) {
		for (int k = 0; k < Hole.cols; k++) {
			EXPECT_LT(std::fabsf(r.at<float>(i, k) - GT.at<float>(i, k)), 0.0001f);
		}
	}
}

TEST(PM, Gradient_CV_32FC2) {
	const std::string TestDir = "Images/Gradient/";

	cv::Mat Hole(cv::Size(101, 101), CV_32FC2);
	cv::Mat GT(cv::Size(101, 101), CV_32FC2);
	cv::Mat Mask(cv::Size(Hole.rows, Hole.cols), CV_8UC1);
	
	for (uint i = 0; i < Hole.rows; i++) {
		for (uint k = 0; k < Hole.cols; k++) {
			if (i <= 50) {
				Hole.at<cv::Vec2f>(i, k) = cv::Vec2f(i * 5, i * 5);
				GT.at<cv::Vec2f>(i, k) = cv::Vec2f(i * 5, i * 5);
			}
			else {
				Hole.at<cv::Vec2f>(i, k) = cv::Vec2f((Hole.rows - 1 - i) * 5, (Hole.rows - 1 - i) * 5);
				GT.at<cv::Vec2f>(i, k) = cv::Vec2f((Hole.rows - 1 - i) * 5, (Hole.rows - 1 - i) * 5);
			}

			if (i >= 40 && i <= 60 && k >= 40 && k <= 60) {
				Mask.at<unsigned char>(i, k) = 255;
				Hole.at<cv::Vec2f>(i, k) = cv::Vec2f(0, 0);
			}
			else {
				Mask.at<unsigned char>(i, k) = 0;
			}

		}
	}

	std::vector<cv::Mat> Channels;
	cv::split(Hole, Channels);
	cv::imwrite(TestDir + "Hole1.png", Channels[0]);
	cv::imwrite(TestDir + "Hole2.png", Channels[1]);

	cv::imwrite(TestDir + "Mask.png", Mask);
	cv::Mat r = PM::run(Hole, Mask, 11);

	Channels.clear();
	cv::split(r, Channels);
	cv::imwrite(TestDir + "r1.png", Channels[0]);
	cv::imwrite(TestDir + "r2.png", Channels[1]);

	for (int i = 0; i < Hole.rows; i++) {
		for (int k = 0; k < Hole.cols; k++) {
			EXPECT_LT(std::fabsf(r.at<float>(i, k) - GT.at<float>(i, k)), 0.0005f);
		}
	}
}

TEST(PM, LoadImage_CV_8UC1) {
	const std::string HoleImagePath = IMAGEINPAINTING_DIR + std::string("2/hole.png");
	const std::string MaskImagePath = IMAGEINPAINTING_DIR + std::string("2/mask.png");

	cv::Mat Hole = cv::imread(HoleImagePath, CV_8UC1);
	cv::Mat Mask = cv::imread(MaskImagePath, CV_8UC1);
	cv::Mat ns, telea, pm, pepm;

	cv::inpaint(Hole, Mask, ns, 1.0, cv::INPAINT_NS);
	cv::inpaint(Hole, Mask, telea, 1.0, cv::INPAINT_TELEA);
	pm = PM::run(Hole, Mask, 11);

	cv::imwrite("ns.png", ns);
	cv::imwrite("telea.png", telea);
	cv::imwrite("pm.png", pm);
}
