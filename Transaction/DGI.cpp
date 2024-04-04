#include "pch.h"
#include "DGI.h"

bool CDGI::setResolution(uint vWorkRes, uint vRecoRes) {
	_EARLY_RETURN(vWorkRes * vRecoRes == 0, "DGI set res error: res == 0.", false);

	m_WorkRes = vWorkRes;
	m_RecoRes = vRecoRes;
	return true;
}

bool CDGI::run(const PC_t::Ptr vInput, PC_t::Ptr& voOutput) {
	_EARLY_RETURN(!isPointCloudValid(vInput), "DGI run error: input is not valid.", false);

	core::CHeightMapGenerator HMGenerator;
	ptr<core::CHeightMap> pHeight = HMGenerator.generate(vInput, m_WorkRes, m_WorkRes);
	_EARLY_RETURN(!pHeight->isValid(), "DGI run error: height map is not valid.", false);
	core::CMapWrapper::saveMapToLocal(pHeight, "Images/Input.png");

	ptr<core::CGradientMap> pGradient = core::MapUtil::geneGradient(pHeight);
	_EARLY_RETURN(!pGradient->isValid(), "DGI run error: gradient map is not valid.", false);

	ptr<core::CMaskMap> pMask = core::MapUtil::geneMask<vec2f>(pGradient);
	_EARLY_RETURN(!pMask->isValid(), "DGI run error: mask map is not valid.", false);

	ptr<core::CGradientMap> pGradientFilled = __inpaintImage(pGradient, pMask);
	_EARLY_RETURN(!pGradientFilled->isValid(), "DGI run error: gradient filled map is not valid.", false);

	ptr<core::CGradientMap> pGog = core::MapUtil::geneGradient(pGradientFilled);
	_EARLY_RETURN(!pGog->isValid(), "DGI run error: gog map is not valid.", false);

	ptr<core::CHeightMap> pHeightFilled = __solveEquations(pHeight, pGradientFilled, pGog);
	_EARLY_RETURN(!pHeightFilled->isValid() || !pHeightFilled->isNoEmpty(), "DGI run error: height filled map is not valid.", false);
	core::CMapWrapper::saveMapToLocal(pHeightFilled, "Images/Output.png");

	ptr<core::CHeightMap> pHeightReco = HMGenerator.generate(vInput, m_RecoRes, m_RecoRes);
	_EARLY_RETURN(!pHeightReco->isValid(), "DGI run error: pHeightReco map is not valid.", false);
	core::CMapWrapper::saveMapToLocal(pHeightReco, "Images/InputReco.png");

	ptr<core::CHeightMap> pFilledReco = core::MapUtil::resize(pHeightFilled, pHeightReco->getWidth(), pHeightReco->getHeight());
	_EARLY_RETURN(!pFilledReco->isValid(), "DGI run error: FilledReco map is not valid.", false);
	core::CMapWrapper::saveMapToLocal(pFilledReco, "Images/OutputReco.png");

	voOutput = __genePointCloud(pHeightReco, pFilledReco, core::PointCloudUtil::calcAABB(vInput), 50);
	_EARLY_RETURN(!isPointCloudValid(voOutput), "DGI run error: output is not valid.", false);

	bool r = __removeExcessPoints(vInput, voOutput);
	_EARLY_RETURN(!isPointCloudValid(voOutput), "DGI run error: output is not valid.", false);

	return true;
}

ptr<core::CGradientMap> CDGI::__inpaintImage(const ptr<core::CGradientMap> vRaw, const ptr<core::CMaskMap> vMask) {
	ptr<core::CGradientMap> pFilled(new core::CGradientMap(vRaw->getWidth(), vRaw->getHeight()));
	
	cv::Mat GradientImage = core::CMapWrapper::castMap2CVMat<vec2f>(vRaw);
	cv::Mat MaskImage = core::CMapWrapper::castMap2CVMat<std::uint8_t>(vMask);
	cv::Mat ResultImage;

	alg::CImageInpainting Inpainter;
	bool r = Inpainter.run(GradientImage, MaskImage, ResultImage, alg::EInpaintMode::TEALA);
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

	std::cout << "DGI: filled height map has " << pFilled->getEmptyCount() << " empty values, max: " << pFilled->getMax() << ", min: " << pFilled->getMin() << std::endl;

	return pFilled;
}

PC_t::Ptr CDGI::__genePointCloud(const ptr<core::CHeightMap> vInput, const ptr<core::CHeightMap> vFilled, const core::SAABB& vBox, int vPointNumberPerPixel) {
	
	core::CHeightMapSampler Sampler;
	std::vector<vec3f> Samples;

	bool r = Sampler.sample(vInput, vFilled, vPointNumberPerPixel, Samples);
	_EARLY_RETURN(!r, "DGI run error: sample fails.", nullptr);

	core::CPCMapper Mapper;
	PC_t::Ptr pCloud = Mapper.map(vBox, Samples);
	_EARLY_RETURN(!isPointCloudValid(pCloud), "DGI run error: new point cloud is invalid.", nullptr);

	return pCloud;
}

bool CDGI::__removeExcessPoints(const PC_t::Ptr vRaw, PC_t::Ptr& vioFilled) {
	core::CDuplicateRemover Remover;
	bool r = Remover.run(vRaw, vioFilled, 5);
	_EARLY_RETURN(!r, "DGI run error: remove excess points fails.", false);

	return true;
}
