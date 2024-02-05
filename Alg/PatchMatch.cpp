#include "pch.h"
#include "PatchMatch.h"

using namespace PM;

/*
  opencv coordinates:
  ------------------->
  |					 x(col)
  |
  |
  |
  ¡ý y(row)
*/

float calcSSIM(const cv::Mat& a, const cv::Mat& b) {
	const double C1 = 6.5025, C2 = 58.5225;
	int d = CV_32F;
	cv::Mat I1, I2;
	a.convertTo(I1, d);
	b.convertTo(I2, d);
	cv::Mat I1_2 = I1.mul(I1);
	cv::Mat I2_2 = I2.mul(I2);
	cv::Mat I1_I2 = I1.mul(I2);
	cv::Mat mu1, mu2;
	GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
	GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);
	cv::Mat mu1_2 = mu1.mul(mu1);
	cv::Mat mu2_2 = mu2.mul(mu2);
	cv::Mat mu1_mu2 = mu1.mul(mu2);
	cv::Mat sigma1_2, sigam2_2, sigam12;
	GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
	sigma1_2 -= mu1_2;

	GaussianBlur(I2_2, sigam2_2, cv::Size(11, 11), 1.5);
	sigam2_2 -= mu2_2;

	GaussianBlur(I1_I2, sigam12, cv::Size(11, 11), 1.5);
	sigam12 -= mu1_mu2;
	cv::Mat t1, t2, t3;
	t1 = 2 * mu1_mu2 + C1;
	t2 = 2 * sigam12 + C2;
	t3 = t1.mul(t2);

	t1 = mu1_2 + mu2_2 + C1;
	t2 = sigma1_2 + sigam2_2 + C2;
	t1 = t1.mul(t2);

	cv::Mat ssim_map;
	divide(t3, t1, ssim_map);
	cv::Scalar mssim = cv::mean(ssim_map);

	double ssim = (mssim.val[0] + mssim.val[1] + mssim.val[2]) / 3;
	return ssim;
}

float computePatchDist(const cv::Mat& vLhs, const cv::Mat& vRhs, int vPatchSize)
{
	int metric = 0;
	float dist = 0.0f;

	if (metric == 0) {
		dist = (float)cv::norm(vLhs, vRhs) * vPatchSize * vPatchSize;
	}
	else if (metric == 1) {
		dist = (float)cv::norm(vLhs, vRhs, cv::NORM_L1) * vPatchSize * vPatchSize;
	}
	else if (metric == 2)
	{
		float ssim = calcSSIM(vLhs, vRhs);
		if (ssim < 0) ssim = 0;
		else if (ssim > 1) ssim = 1;
		dist = (1.0f / ssim) * vPatchSize * vPatchSize;
	}

	return dist;
}

void propagate(const cv::Mat& vSource, const cv::Mat& vTarget, const cv::Mat& vMask, const std::pair<int, int>& vPos /* row, col */, const std::pair<int, int>& vPosGuess /* row, col */, int vPatchSize, cv::Mat vioNN)
{
	if (vPos.first == vPosGuess.first && vPos.second == vPosGuess.second) return;

	cv::Rect TargetRect(vPosGuess.second, vPosGuess.first, vPatchSize, vPatchSize);
	cv::Mat MaskPatch = vMask(TargetRect);

	int InvalidNumber = 0;
	for (int i = 0; i < MaskPatch.rows; i++)
		for (int k = 0; k < MaskPatch.cols; k++)
			if (MaskPatch.at<unsigned char>(i, k))
				InvalidNumber++;
	if (InvalidNumber * 10 > MaskPatch.rows * MaskPatch.cols) return;

	cv::Mat SourcePatch = vSource(cv::Rect(vPos.second, vPos.first, vPatchSize, vPatchSize));
	cv::Mat TargetPatch = vTarget(TargetRect);
	float Dist = computePatchDist(SourcePatch, TargetPatch, vPatchSize);
	if (Dist < vioNN.at<cv::Vec3f>(vPos.first, vPos.second)[2])
	{
		vioNN.at<cv::Vec3f>(vPos.first, vPos.second)[0] = vPosGuess.second;
		vioNN.at<cv::Vec3f>(vPos.first, vPos.second)[1] = vPosGuess.first;
		vioNN.at<cv::Vec3f>(vPos.first, vPos.second)[2] = Dist;
	}
}

cv::Mat PM::PatchMatch(const cv::Mat& vSource, const cv::Mat& vTarget, const cv::Mat& vMask, int vPatchSize)
{
	cv::Mat NN = cv::Mat::zeros(vSource.size(), CV_32FC3);		/* Col, Row, Dist */

	for (int i = 0; i < vSource.rows - vPatchSize + 1; i++)
		for (int k = 0; k < vSource.cols - vPatchSize + 1; k++)
		{
			NN.at<cv::Vec3f>(i, k)[0] = MathUtil::geneRandomInt(0, vTarget.cols - vPatchSize);
			NN.at<cv::Vec3f>(i, k)[1] = MathUtil::geneRandomInt(0, vTarget.rows - vPatchSize);

			cv::Mat SourcePatch = vSource(cv::Rect(k, i, vPatchSize, vPatchSize));
			cv::Mat TargetPatch = vTarget(cv::Rect(NN.at<cv::Vec3f>(i, k)[0], NN.at<cv::Vec3f>(i, k)[1], vPatchSize, vPatchSize));

			NN.at<cv::Vec3f>(i, k)[2] = computePatchDist(SourcePatch, TargetPatch, vPatchSize);
			//std::cout << "[" << i << ", " << k << "] Patch Dist: " << NN.at<cv::Vec3f>(i, k)[2] << std::endl;
		}

	int MaxIters = 5;
	for (int Iter = 0; Iter < MaxIters; Iter++)
	{
		int RowStart, RowEnd, ColStart, ColEnd, Step;
		if (Iter % 2)
		{
			RowStart = vSource.rows - vPatchSize - 1;
			RowEnd = -1;
			ColStart = vSource.cols - vPatchSize - 1;
			ColEnd = -1;
			Step = -1;
		}
		else
		{
			RowStart = 1;
			RowEnd = vSource.rows - vPatchSize + 1;
			ColStart = 1;
			ColEnd = vSource.cols - vPatchSize + 1;
			Step = 1;
		}

		for (int i = RowStart; i != RowEnd; i += Step)
			for (int k = ColStart; k != ColEnd; k += Step)
			{
				if (k < vSource.cols - vPatchSize + Step)
				{
					std::pair<int, int> PosGuess(NN.at<cv::Vec3f>(i, k - Step)[1], NN.at<cv::Vec3f>(i, k - Step)[0] + Step);	 /* row, col */

					if (PosGuess.second <= vTarget.cols - vPatchSize && PosGuess.second >= 0)
						propagate(vSource, vTarget, vMask, std::make_pair(i, k), PosGuess, vPatchSize, NN);
				}

				if (i < vSource.rows - vPatchSize + Step)
				{
					std::pair<int, int> PosGuess(NN.at<cv::Vec3f>(i - Step, k)[1] + Step, NN.at<cv::Vec3f>(i - Step, k)[0]);	 /* row, col */

					if (PosGuess.first <= vTarget.rows - vPatchSize && PosGuess.first >= 0)
						propagate(vSource, vTarget, vMask, std::make_pair(i, k), PosGuess, vPatchSize, NN);
				}

				int CurX = NN.at<cv::Vec3f>(i, k)[0];
				int CurY = NN.at<cv::Vec3f>(i, k)[1];

				for (int Range = std::max(vTarget.rows, vTarget.cols); Range >= 1; Range /= 2)
				{
					int RandomRow = MathUtil::geneRandomInt(std::max(CurY - Range, 0), std::min(CurY + Range + 1, vTarget.rows - vPatchSize - 1));
					int RandomCol = MathUtil::geneRandomInt(std::max(CurX - Range, 0), std::min(CurX + Range + 1, vTarget.cols - vPatchSize - 1));

					propagate(vSource, vTarget, vMask, std::make_pair(i, k), std::make_pair(RandomRow, RandomCol), vPatchSize, NN);
				}
			}
	}

	return NN;
}
