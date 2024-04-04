#include "header.h"
#include "task.h"

cv::Mat Task::generateMask(const std::string& vHolePath) {
	cv::Mat Hole = cv::imread(vHolePath, 0);
	cv::Mat Mask(Hole.size(), CV_8UC1);

	for (int i = 0; i < Hole.rows; i++) {
		for (int k = 0; k < Hole.cols; k++) {
			if (Hole.at<uchar>(i, k) == 0) {
				Mask.at<uchar>(i, k) = (uchar)(255);
			}
			else {
				Mask.at<uchar>(i, k) = 0;
			}
		}
	}

	return Mask;
}

cv::Mat Task::generateMask(const std::string& vHolePath, const std::string& vGTPath) {
	cv::Mat Hole = cv::imread(vHolePath, 0);
	cv::Mat GT = cv::imread(vGTPath, 0);
	cv::Mat Mask(Hole.size(), CV_8UC1);

	for (int i = 0; i < Hole.rows; i++) {
		for (int k = 0; k < Hole.cols; k++) {
			if (Hole.at<uchar>(i, k) != GT.at<uchar>(i, k)) {
				Mask.at<uchar>(i, k) = (uchar)(255);
			}
			else {
				Mask.at<uchar>(i, k) = 0;
			}
		}
	}

	return Mask;
}

cv::Mat Task::mergeImage(const cv::Mat& a, const cv::Mat& b, const cv::Mat& vMask) {
	cv::Mat Merge = a.clone();
	cv::Mat Rate(Merge.size(), CV_32FC1);

	for (int i = 0; i < Rate.rows; i++) {
		for (int k = 0; k < Rate.cols; k++) {
			Rate.at<float>(i, k) = MathUtil::geneRandomReal(0.5, 0.9);
		}
	}

	cv::Mat RateBlur;
	cv::blur(Rate, RateBlur, cv::Size(10, 10));
	cv::blur(Rate, RateBlur, cv::Size(10, 10));


	cv::Mat Weights(a.size(), CV_32F);
	cv::distanceTransform(vMask, Weights, cv::DIST_L2, 5);
	
	for (int i = 0; i < Merge.rows; i++) {
		for (int k = 0; k < Merge.cols; k++) {
			if (vMask.at<uchar>(i, k) == 255) {
				float weight = 1 / Weights.at<float>(i, k);
				float ra = 0.2f + 0.8 * weight;
				//float ra = Rate.at<float>(i, k);
				std::cout << ra << std::endl;
				Merge.at<uchar>(i, k) = (uchar)((float)a.at<uchar>(i, k) * ra + (float)b.at<uchar>(i, k) * (1 - ra));
			}
		}
	}
	return Merge;
}

template <typename T>
using ptr = std::shared_ptr<T>;
using hmptr = ptr<core::CHeightMap>;
using mmptr = ptr<core::CMaskMap>;
using gmptr = ptr<core::CGradientMap>;

cv::Mat Task::reconstruct(const cv::Mat& vGT, const cv::Mat& vMask) {
	cv::Mat GT;
	vGT.convertTo(GT, CV_32FC1);

	hmptr pHeight = std::get<0>(core::CMapWrapper::castCVMat2Map(GT));
	gmptr pGradient = core::MapUtil::geneGradient(pHeight);
	mmptr pMask = std::get<2>(core::CMapWrapper::castCVMat2Map(vMask));
	gmptr pGoG = core::MapUtil::geneGradient(pGradient);

	for (int i = 0; i < pHeight->getWidth(); i++) {
		for (int k = 0; k < pHeight->getHeight(); k++) {
			if (pMask->isEmpty(i, k)) {
				pHeight->setEmpty(i, k);
			}
		}
	}

	cv::imwrite("Images/withhole.png", core::CMapWrapper::castMap2CVMat<float>(pHeight));

	core::CSolverBuilder Builder;
	bool r = Builder.run(pHeight, pGoG, pGradient);

	Eigen::MatrixXf Coeff, ConstNumbers, Solutions;
	Builder.dumpMatrix(Coeff, ConstNumbers);

	std::vector<Eigen::Vector2f> Unknowns;
	Builder.dumpUnknowns(Unknowns);

	core::CSparseLinearSolver Solver(Coeff, ConstNumbers);
	Solutions = Solver.solve(core::ESolverMode::FullPivLU);

	for (int i = 0; i < Unknowns.size(); i++) {
		pHeight->setValue(Unknowns[i][0], Unknowns[i][1], Solutions(i, 0));
	}

	cv::Mat Res = core::CMapWrapper::castMap2CVMat<float>(pHeight);

	return Res;
}

cv::Mat Task::inpaintImage(const cv::Mat& vSrc, const cv::Mat& vMask, const cv::Mat& vGT) {
	cv::Mat Res;

	alg::CPoissonImageInpainting Inpainter;
	Inpainter.setGT(vGT);
	Inpainter.run(vSrc, vMask, Res, alg::EPoissonGradient::MIX);

	return Res;
}
