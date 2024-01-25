#include "pch.h"
#include "DGI.h"

bool CDGI::setResolution(uint vRes) {
	_EARLY_RETURN(vRes == 0, "DGI set res error: res == 0.", false);

	m_Res = vRes;
	return true;
}

bool CDGI::run(const PC_t::Ptr vInput, PC_t::Ptr voOutput) {
	_EARLY_RETURN(!isPointCloudValid(vInput), "DGI run error: input is not valid.", false);

	core::CHeightMapGenerator HMGenerator;
	ptr<core::CHeightMap> pHeight = HMGenerator.generate(vInput, m_Res, m_Res);
	_EARLY_RETURN(!pHeight->isValid(), "DGI run error: height map is not valid.", false);

	ptr<core::CGradientMap> pGradient = core::MapUtil::geneGradient(pHeight);
	_EARLY_RETURN(!pGradient->isValid(), "DGI run error: gradient map is not valid.", false);

	ptr<core::CMaskMap> pMask = core::MapUtil::geneMask<vec2f>(pGradient);
	_EARLY_RETURN(!pMask->isValid(), "DGI run error: mask map is not valid.", false);

	ptr<core::CGradientMap> pGradientFilled = __inpaintImage(pGradient, pMask);
	_EARLY_RETURN(!pGradientFilled->isValid(), "DGI run error: gradient filled map is not valid.", false);

	ptr<core::CGradientMap> pGog = core::MapUtil::geneGradient(pGradientFilled);
	_EARLY_RETURN(!pGog->isValid(), "DGI run error: gog map is not valid.", false);

	ptr<core::CHeightMap> pHeightFilled = __solveEquations(pHeight, pGradientFilled, pGog);
	_EARLY_RETURN(!pHeightFilled->isValid(), "DGI run error: height filled map is not valid.", false);

	voOutput = __genePointCloud(pHeight, pHeightFilled, 10);
	_EARLY_RETURN(!isPointCloudValid(voOutput), "DGI run error: output is not valid.", false);

	return false;
}

ptr<core::CGradientMap> CDGI::__inpaintImage(const ptr<core::CGradientMap> vRaw, const ptr<core::CMaskMap> vMask) {
	ptr<core::CGradientMap> pFilled(new core::CGradientMap(vRaw->getWidth(), vRaw->getHeight()));
	
	cv::Mat GradientImage = core::CMapWrapper::castMap2CVMat<vec2f>(vRaw);
	cv::Mat MaskImage = core::CMapWrapper::castMap2CVMat<std::uint8_t>(vMask);
	cv::Mat ResultImage;

	alg::CImageInpainting Inpainter;
	bool r = Inpainter.run(GradientImage, MaskImage, ResultImage, alg::PM);
	_EARLY_RETURN(!r, "DGI run error: image inpainting fails.", pFilled);

	pFilled = std::get<1>(core::CMapWrapper::castCVMat2Map(ResultImage));
	_EARLY_RETURN(!pFilled->isValid(), "DGI run error: cast cv mat 2 map fails.", pFilled);

	return pFilled;
}

ptr<core::CHeightMap> CDGI::__solveEquations(const ptr<core::CHeightMap> vInput, const ptr<core::CGradientMap> vGradientFilled, const ptr<core::CGradientMap> vGog) {
	ptr<core::CHeightMap> pFilled(new core::CHeightMap(vInput->getWidth(), vInput->getHeight()));
	pFilled->set(vInput);

	core::CSolverBuilder Builder;
	bool r = Builder.run(vInput, vGog, vGradientFilled);
	_EARLY_RETURN(!r, "DGI run error: solver builder fails.", pFilled);

	Eigen::MatrixXf Coeff, ConstNumbers, Solutions;
	Builder.dumpMatrix(Coeff, ConstNumbers);
	
	std::vector<Eigen::Vector2f> Unknowns;
	Builder.dumpUnknowns(Unknowns);

	core::CSparseLinearSolver Solver(Coeff, ConstNumbers);
	Solutions = Solver.solve(core::ESolverMode::FullPivLU);
	_EARLY_RETURN(!core::EigenUtil::isEigenValid(Solutions) || core::EigenUtil::isEigenHasEmptyValue(Solutions), "DGI run error: solve equation fails.", pFilled);

	for (int i = 0; i < Unknowns.size(); i++) {
		pFilled->setValue(Unknowns[i][0], Unknowns[i][1], Solutions(i, 0));
	}

	_EARLY_RETURN(!pFilled->isValid(), "DGI run error: filled height map set value bug.", pFilled);

	return pFilled;
}

PC_t::Ptr CDGI::__genePointCloud(const ptr<core::CHeightMap> vInput, const ptr<core::CHeightMap> vFilled, int vPointNumberPerPixel) {
	
	core::CHeightMap2PCMapper Mapper;
	PC_t::Ptr pNew = Mapper.map2PC(vInput, vFilled, vPointNumberPerPixel);
	_EARLY_RETURN(!isPointCloudValid(pNew), "DGI run error: new point cloud is invalid.", pNew);

	return pNew;
}


