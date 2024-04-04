#include "pch.h"
#include "PoissonImageInpainting.h"
#include "ImageUtil.h"
#include "HeightMap.h"
#include "GradientMap.h"
#include "MaskMap.h"
#include "MapUtil.h"
#include "MapWrapper.h"
#include "PMInterface.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"
#include "EigenUtil.h"
#include "MixInpainting.h"
#include "CVInpainting.h"

using namespace alg;

template <typename T>
using ptr = std::shared_ptr<T>;
using hmptr = ptr<core::CHeightMap>;
using mmptr = ptr<core::CMaskMap>;
using gmptr = ptr<core::CGradientMap>;

bool CPoissonImageInpainting::run(const cv::Mat& vSrc, const cv::Mat& vMask, cv::Mat& voRes, EPoissonGradient vMode) {
	_EARLY_RETURN(ImageUtil::isImageInpaintingInputValid(vSrc, vMask) == false, "", false);

	int ImageType = vSrc.type();
	if (ImageType != CV_32FC1) {
		vSrc.convertTo(m_Work, CV_32FC1);
	}
	else {
		m_Work = vSrc.clone();
	}

	hmptr pHeight = std::get<0>(core::CMapWrapper::castCVMat2Map(m_Work));

	for (uint i = 0; i < pHeight->getWidth(); i++) {
		for (uint k = 0; k < pHeight->getWidth(); k++) {
			if (vMask.at<uchar>(i, k) == 255) {
				pHeight->setEmpty(i, k);
			}
		}
	}

	gmptr pGradient = core::MapUtil::geneGradient(pHeight);
	mmptr pMask = core::MapUtil::geneMask<vec2f>(pGradient);

	cv::Mat GradientImage = core::CMapWrapper::castMap2CVMat<vec2f>(pGradient);
	cv::Mat MaskImage = core::CMapWrapper::castMap2CVMat<std::uint8_t>(pMask);
	cv::Mat GradientResult;
	
	switch (vMode) {
	case EPoissonGradient::PM:
	{
		/*CMixInpainting Inpainter;
		Inpainter.setDistThres(0);
		Inpainter.run(GradientImage, MaskImage, GradientResult);*/
		GradientResult = PM::run(GradientImage, MaskImage, 11);
		break;
	}
	case EPoissonGradient::MIX:
	{
		CMixInpainting Inpainter;
		Inpainter.setDistThres(2);
		Inpainter.setCVInpaintingType(ECVInpaintingType::TELEA);
		Inpainter.run(GradientImage, MaskImage, GradientResult);
		break;
	}
	case EPoissonGradient::NS:
	{
		CCVInpainting Inpainter;
		Inpainter.run(GradientImage, MaskImage, GradientResult, ECVInpaintingType::NS);
		break;
	}
	case EPoissonGradient::TELEA:
	{
		CCVInpainting Inpainter;
		Inpainter.run(GradientImage, MaskImage, GradientResult, ECVInpaintingType::TELEA);
		break;
	}
	case EPoissonGradient::EB:
	{
		break;
	}
	case EPoissonGradient::GB:
	{
		break;
	}
	default:
		break;
	}

	if (m_GT.data) {
		__mergeGT(GradientResult);
	}

	gmptr pGradientFilled = std::get<1>(core::CMapWrapper::castCVMat2Map(GradientResult));
	gmptr pGoG = core::MapUtil::geneGradient(pGradientFilled);

	core::CSolverBuilder Builder;
	bool r = Builder.run(pHeight, pGoG, pGradientFilled);
	_EARLY_RETURN(!r, "Poisson error: solver builder fails.", false);

	Eigen::MatrixXf Coeff, ConstNumbers, Solutions;
	Builder.dumpMatrix(Coeff, ConstNumbers);

	std::vector<Eigen::Vector2f> Unknowns;
	Builder.dumpUnknowns(Unknowns);

	core::CSparseLinearSolver Solver(Coeff, ConstNumbers);
	Solutions = Solver.solve(core::ESolverMode::FullPivLU);
	_EARLY_RETURN(!core::EigenUtil::isEigenValid(Solutions) || core::EigenUtil::isEigenHasEmptyValue(Solutions), "Poisson error: solve equation fails.", false);

	for (int i = 0; i < Unknowns.size(); i++) {
		pHeight->setValue(Unknowns[i][0], Unknowns[i][1], Solutions(i, 0));
	}

	_EARLY_RETURN(!pHeight->isValid(), "Poisson error: filled height map set value bug.", false);
	std::cout << "PCID: filled height map has " << pHeight->getEmptyCount() << " empty values, max: " << pHeight->getMax() << ", min: " << pHeight->getMin() << std::endl;

	voRes = core::CMapWrapper::castMap2CVMat<float>(pHeight);
	
	return true;
}

void CPoissonImageInpainting::__mergeGT(cv::Mat& vioGradientResult) {
	cv::imwrite("Images/Merge/gt.png", m_GT);

	cv::Mat WorkGT;
	m_GT.convertTo(WorkGT, CV_32FC1);
	hmptr pHeight = std::get<0>(core::CMapWrapper::castCVMat2Map(WorkGT));
	gmptr pGradient = core::MapUtil::geneGradient(pHeight);
	cv::Mat GradientGT = core::CMapWrapper::castMap2CVMat<vec2f>(pGradient);

	std::vector<cv::Mat> GTChannels, RawChannels, MergeChannels;
	cv::split(GradientGT, GTChannels);
	cv::split(vioGradientResult, RawChannels);

	for (int i = 0; i < vioGradientResult.rows; i++) {
		for (int k = 0; k < vioGradientResult.cols; k++) {
			vioGradientResult.at<cv::Vec2f>(i, k) = (vioGradientResult.at<cv::Vec2f>(i, k) * 0 + GradientGT.at<cv::Vec2f>(i, k) * 2) / 2;
		}
	}

	cv::split(vioGradientResult, MergeChannels);

	ImageUtil::saveToLocal(GTChannels[0], "Images/Merge/GTChannel-0.png", true);
	ImageUtil::saveToLocal(GTChannels[1], "Images/Merge/GTChannel-1.png", true);
	ImageUtil::saveToLocal(RawChannels[0], "Images/Merge/RawChannel-0.png", true);
	ImageUtil::saveToLocal(RawChannels[1], "Images/Merge/RawChannel-1.png", true);
	ImageUtil::saveToLocal(MergeChannels[0], "Images/Merge/MergeChannel-0.png", true);
	ImageUtil::saveToLocal(MergeChannels[1], "Images/Merge/MergeChannel-1.png", true);
}
