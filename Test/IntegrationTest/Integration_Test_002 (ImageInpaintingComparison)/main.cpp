
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
#include <variant>

/* BOOST */
#include <boost/format.hpp>

/* CV */
#include <opencv2/opencv.hpp>

/* COMMON */
#include "Common.h"

/* ALG */
#include "ImageInpainting.h"
#include "ImageUtil.h"

int main() {

	const std::string ImagePath = IMAGEINPAINTING_DIR + std::string("4/hole.png");
	cv::Mat Input = cv::imread(ImagePath, cv::IMREAD_GRAYSCALE);
	cv::Mat Mask(Input.size(), CV_8UC1);
	
	for (int i = 0; i < Input.rows; i++) {
		for (int k = 0; k < Input.cols; k++) {
			if (Input.at<uchar>(i, k) == 0) {
				Mask.at<uchar>(i, k) = 255;
			}
			else {
				Mask.at<uchar>(i, k) = 0;
			}
		}
	}

	cv::imwrite("3-mask.png", Mask);

	return 0;
}





