#include "pch.h"
#include "PMInterface.h"
#include "Inpaint.h"

using namespace PM;

/*
Mask: 
	0 raw; 
	1 to be filled
*/
cv::Mat PM::run(const cv::Mat& vRaw, const cv::Mat& vMask, int vPatchSize /*= 11*/)
{
	_ASSERTE(vPatchSize > 0);
	_ASSERTE(vRaw.data && vMask.data);
	_ASSERTE(vRaw.rows == vMask.rows && vRaw.cols == vMask.cols);

	cv::Mat Result = vRaw.clone();
	if (vRaw.type() == 5)		// CV_32FC1
	{
		CInpainter Inpainter(vRaw, vMask, vPatchSize, 3, 2);
		Inpainter.inpaint();
		Inpainter.dumpResult(Result);
	}
	else if (vRaw.type() == 13)	// CV_32FC2
	{
		CInpainter Inpainter(vRaw, vMask, vPatchSize, 3, 2);
		Inpainter.inpaint();
		Inpainter.dumpResult(Result);
	}
	else if (vRaw.type() == 0)	// CV_8UC1
	{
		cv::Mat Raw(vRaw.size(), CV_32FC1), TempRes;
		for (int i = 0; i < Raw.rows; i++) {
			for (int k = 0; k < Raw.cols; k++) {
				Raw.at<float>(i, k) = float(vRaw.at<uchar>(i, k));
			}
		}
		CInpainter Inpainter(Raw, vMask, vPatchSize, 3, 2);
		Inpainter.inpaint();
		Inpainter.dumpResult(TempRes);
		
		for (int i = 0; i < Raw.rows; i++) {
			for (int k = 0; k < Raw.cols; k++) {
				Result.at<uchar>(i, k) = uchar(TempRes.at<float>(i, k));
			}
		}
	}

	return Result;
}
