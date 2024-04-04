#include "pch.h"
#include "ImageUtil.h"

float ImageUtil::calcSSIM(const cv::Mat& a, const cv::Mat& b) {
	const float C1 = 6.5025f, C2 = 58.5225f;
	int d = CV_32F;
	cv::Mat I1, I2;
	a.convertTo(I1, d);
	b.convertTo(I2, d);
	cv::Mat I1_2 = I1.mul(I1);
	cv::Mat I2_2 = I2.mul(I2);
	cv::Mat I1_I2 = I1.mul(I2);
	cv::Mat mu1, mu2;
	cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5f);
	cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5f);
	cv::Mat mu1_2 = mu1.mul(mu1);
	cv::Mat mu2_2 = mu2.mul(mu2);
	cv::Mat mu1_mu2 = mu1.mul(mu2);
	cv::Mat sigma1_2, sigam2_2, sigam12;
	cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5f);
	sigma1_2 -= mu1_2;

	cv::GaussianBlur(I2_2, sigam2_2, cv::Size(11, 11), 1.5f);
	sigam2_2 -= mu2_2;

	cv::GaussianBlur(I1_I2, sigam12, cv::Size(11, 11), 1.5f);
	sigam12 -= mu1_mu2;
	cv::Mat t1, t2, t3;
	t1 = 2 * mu1_mu2 + C1;
	t2 = 2 * sigam12 + C2;
	t3 = t1.mul(t2);

	t1 = mu1_2 + mu2_2 + C1;
	t2 = sigma1_2 + sigam2_2 + C2;
	t1 = t1.mul(t2);

	cv::Mat ssim_map;
	cv::divide(t3, t1, ssim_map);
	cv::Scalar mssim = cv::mean(ssim_map);

	float ssim = (mssim.val[0] + mssim.val[1] + mssim.val[2]) / 3;
	return ssim;
}

bool ImageUtil::isEmptyInNeighbor(const cv::Mat& m, uint x, uint y) {
	_EARLY_RETURN(x >= m.rows || y >= m.cols, "Index error", false);

	if (x >= 1) {
		if (m.at<uchar>(x - 1, y) == 1) {
			return true;
		}
	}
	if (y >= 1) {
		if (m.at<uchar>(x, y - 1) == 1) {
			return true;
		}
	}
	if (x <= m.rows - 2) {
		if (m.at<uchar>(x + 1, y) == 1) {
			return true;
		}
	}
	if (y <= m.cols - 2) {
		if (m.at<uchar>(x, y + 1) == 1) {
			return true;
		}
	}

	return false;
}

bool ImageUtil::isImageValid(const cv::Mat& m) {
	return !m.empty();
}

bool ImageUtil::isImageInpaintingInputValid(const cv::Mat& vSrc, const cv::Mat& vMask) {
	_EARLY_RETURN(ImageUtil::isImageValid(vSrc) == false || ImageUtil::isImageValid(vMask) == false, "Input is invalid.", false);
	_EARLY_RETURN(vSrc.rows != vMask.rows || vSrc.cols != vMask.cols, "Input size is not same.", false);
	_EARLY_RETURN(vMask.type() != CV_8UC1, "Mask is not CV_8UC1.", false);
	return true;
}

bool ImageUtil::saveToLocal(const cv::Mat& vSrc, const std::string& vPath, bool vIsNormalize) {
	cv::Mat Save = vSrc.clone();

	if (vIsNormalize) {
		std::vector<cv::Mat> Channels;
		cv::split(Save, Channels);
		for (int i = 0; i < Channels.size(); i++) {
			cv::Mat Reshape = Channels[i].reshape(1);
			double MinValue, MaxValue;
			cv::Point MinIdx, MaxIdx;
			cv::minMaxLoc(Reshape, &MinValue, &MaxValue, &MinIdx, &MaxIdx);
			double Span = MaxValue - MinValue;

			std::cout << "Max: " << MaxValue << ", Min: " << MinValue << std::endl;

			for (int m = 0; m < Channels[i].rows; m++) {
				for (int n = 0; n < Channels[i].cols; n++) {
					if (Channels[i].type() == CV_8UC1) {
						Channels[i].at<uchar>(m, n) = ((double)Channels[i].at<uchar>(m, n) - MinValue) / Span * 255;
					}
					else if (Channels[i].type() == CV_32FC1) {
						Channels[i].at<float>(m, n) = ((double)Channels[i].at<float>(m, n) - MinValue) / Span * 255;
					}
				}
			}
		}
		cv::merge(Channels, Save);
	}

	if (Save.channels() == 1) {
		cv::imwrite(vPath, Save);
	}
	else {
		std::vector<cv::Mat> Channels;
		cv::split(Save, Channels);
		for (int i = 0; i < Channels.size(); i++) {
			cv::imwrite(vPath + "-" + std::to_string(i) + ".png", Channels[i]);
		}
	}

	return true;
}

bool ImageUtil::isIndexValid(const cv::Mat& m, int x, int y) {
	if (x < 0 || y < 0 || x >= m.rows || y > m.cols) {
		return false;
	}

	return true;
}

bool ImageUtil::isHoleBoundary(const cv::Mat& m, uint x, uint y, int d /* = 1 */) {
	_EARLY_RETURN(x >= m.rows || y >= m.cols, "Index error", false);

	if (m.at<uchar>(x, y) == 0) {
		return false;
	}

	for (int i = (int)x - d; i <= (int)x + d; i++) {
		for (int k = (int)y - d; k <= (int)y + d; k++) {
			if (isIndexValid(m, i, k) == false) {
				return true;
			}

			if (std::abs(i - (int)x) + std::abs(k - (int)y) > d) {
				continue;
			}

			if (m.at<uchar>(i, k) == 0) {
				return true;
			}
		}
	}

	return false;
}
