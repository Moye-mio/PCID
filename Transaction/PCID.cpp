#include "pch.h"
#include "PCID.h"

bool CPCID::setResolution(uint vWorkRes, uint vRecoRes) {
	_EARLY_RETURN(vWorkRes * vRecoRes == 0, "PCID set res error: res == 0.", false);

	m_WorkRes = vWorkRes;
	m_RecoRes = vRecoRes;
	return true;
}

bool CPCID::run(const PC_t::Ptr vInput, const PC_t::Ptr vSub, PC_t::Ptr& voOutput) {
	_EARLY_RETURN(!isPointCloudValid(vInput) || !isPointCloudValid(vSub), "PCID error: input or sub is not valid.", false);

	core::CNurbsFitting Fitting;
	Fitting.setCVSavePath("ControlPts/pts.txt");
	Fitting.setKnotSavePath("ControlPts/knots.txt");
	bool r = Fitting.run(vSub, 3, 5, 1);
	const auto Fit = Fitting.getFit();
	_EARLY_RETURN(!r || !Fit, "PCID error: fitting nurbs fails.", false);

	std::vector<SProj> Projs;
	r = core::CSurfaceUtil::calcProjPoints(Fit, vInput, Projs);
	_EARLY_RETURN(!r, "PCID error: calc proj points fails.", false);

	core::CProjManager ProjManager;
	r = ProjManager.calcDirection(Projs);
	_EARLY_RETURN(!r, "PCID error: calc proj direction fails.", false);

	std::vector<vec3f> BriefProjs;
	__prepareHeightMapData(Projs, BriefProjs);

	core::CHeightMapGenerator HMGenerator;
	ptr<core::CHeightMap> pHeight = HMGenerator.generate(BriefProjs, m_WorkRes, m_WorkRes);
	_EARLY_RETURN(!pHeight->isValid(), "PCID error: height map is not valid.", false);
	std::cout << "HeightMap max: " << pHeight->getMax() << ", min: " << pHeight->getMin() << std::endl;

	int DenoiseThres = 10;
	ptr<core::CHeightMap> pHeightCopy = pHeight;
	pHeight = __denoiseHeightMap(pHeight, DenoiseThres);
	_EARLY_RETURN(!pHeight->isValid(), "PCID error: height map is not valid.", false);

	std::cout << "HeightMap max: " << pHeight->getMax() << ", min: " << pHeight->getMin() << std::endl;

	core::CMapWrapper::saveMapToLocal(pHeight, "Images/Input.png");

	ptr<core::CGradientMap> pGradient = core::MapUtil::geneGradient(pHeight);
	_EARLY_RETURN(!pGradient->isValid(), "PCID error: gradient map is not valid.", false);

	ptr<core::CMaskMap> pMask = core::MapUtil::geneMask<vec2f>(pGradient);
	_EARLY_RETURN(!pMask->isValid(), "PCID error: mask map is not valid.", false);

	ptr<core::CGradientMap> pGradientFilled = __inpaintImage(pGradient, pMask);
	_EARLY_RETURN(!pGradientFilled->isValid(), "PCID error: gradient filled map is not valid.", false);

	ptr<core::CGradientMap> pGog = core::MapUtil::geneGradient(pGradientFilled);
	_EARLY_RETURN(!pGog->isValid(), "PCID error: gog map is not valid.", false);

	ptr<core::CHeightMap> pHeightFilled = __solveEquations(pHeight, pGradientFilled, pGog);
	_EARLY_RETURN(!pHeightFilled->isValid() || !pHeightFilled->isNoEmpty(), "PCID error: height filled map is not valid.", false);
	__recoverHeightMap(pHeightFilled, pHeightCopy);
	pHeightFilled = pHeightCopy;
	core::CMapWrapper::saveMapToLocal(pHeightFilled, "Images/Output.png");

	ptr<core::CHeightMap> pHeightReco = HMGenerator.generate(BriefProjs, m_RecoRes, m_RecoRes);
	_EARLY_RETURN(!pHeightReco->isValid(), "PCID error: pHeightReco map is not valid.", false);
	core::CMapWrapper::saveMapToLocal(pHeightReco, "Images/InputReco.png");

	ptr<core::CHeightMap> pFilledReco = core::MapUtil::resize(pHeightFilled, pHeightReco->getWidth(), pHeightReco->getHeight());
	_EARLY_RETURN(!pFilledReco->isValid(), "PCID error: FilledReco map is not valid.", false);
	core::CMapWrapper::saveMapToLocal(pFilledReco, "Images/OutputReco.png");

	voOutput = __genePointCloud(Fit, pHeightReco, pFilledReco, core::PointCloudUtil::calcAABB(vInput), 2);
	_EARLY_RETURN(!isPointCloudValid(voOutput), "PCID error: output is not valid.", false);

	r = __removeExcessPoints(vInput, voOutput);
	_EARLY_RETURN(!isPointCloudValid(voOutput), "PCID error: output is not valid.", false);

	return true;
}

void CPCID::__prepareHeightMapData(const std::vector<SProj>& vProjs, std::vector<vec3f>& voBriefProjs) {
	if (voBriefProjs.size()) {
		voBriefProjs.clear();
	}
	
	float maxd = -FLT_MAX, mind = FLT_MAX;
	for (const auto& p : vProjs) {
		voBriefProjs.emplace_back(vec3f {p.uv.x, p.uv.y, p.dist});
		maxd = std::fmaxf(maxd, p.dist);
		mind = std::fminf(mind, p.dist);
	}

	std::cout << "generate " << voBriefProjs.size() << " brief projs from " << vProjs.size() << " projs" << std::endl;
	std::cout << "proj: max dist: " << maxd << ", mind: " << mind << std::endl;
}

ptr<core::CGradientMap> CPCID::__inpaintImage(const ptr<core::CGradientMap> vRaw, const ptr<core::CMaskMap> vMask) {
	ptr<core::CGradientMap> pFilled(new core::CGradientMap(vRaw->getWidth(), vRaw->getHeight()));

	cv::Mat GradientImage = core::CMapWrapper::castMap2CVMat<vec2f>(vRaw);
	cv::Mat MaskImage = core::CMapWrapper::castMap2CVMat<std::uint8_t>(vMask);
	cv::Mat ResultImage;

	alg::CImageInpainting Inpainter;
	bool r = Inpainter.run(GradientImage, MaskImage, ResultImage, alg::PM);
	_EARLY_RETURN(!r, "PCID error: image inpainting fails.", pFilled);

	pFilled = std::get<1>(core::CMapWrapper::castCVMat2Map(ResultImage));
	_EARLY_RETURN(!pFilled->isValid(), "PCID error: cast cv mat 2 map fails.", pFilled);

	return pFilled;
}

ptr<core::CHeightMap> CPCID::__solveEquations(const ptr<core::CHeightMap> vInput, const ptr<core::CGradientMap> vGradientFilled, const ptr<core::CGradientMap> vGog) {
	ptr<core::CHeightMap> pFilled(new core::CHeightMap(vInput->getWidth(), vInput->getHeight()));
	pFilled->set(vInput);

	core::CSolverBuilder Builder;
	bool r = Builder.run(vInput, vGog, vGradientFilled);
	_EARLY_RETURN(!r, "PCID error: solver builder fails.", pFilled);

	Eigen::MatrixXf Coeff, ConstNumbers, Solutions;
	Builder.dumpMatrix(Coeff, ConstNumbers);

	std::vector<Eigen::Vector2f> Unknowns;
	Builder.dumpUnknowns(Unknowns);

	core::CSparseLinearSolver Solver(Coeff, ConstNumbers);
	Solutions = Solver.solve(core::ESolverMode::FullPivLU);
	_EARLY_RETURN(!core::EigenUtil::isEigenValid(Solutions) || core::EigenUtil::isEigenHasEmptyValue(Solutions), "PCID error: solve equation fails.", pFilled);

	for (int i = 0; i < Unknowns.size(); i++) {
		pFilled->setValue(Unknowns[i][0], Unknowns[i][1], Solutions(i, 0));
	}

	_EARLY_RETURN(!pFilled->isValid(), "PCID error: filled height map set value bug.", pFilled);

	std::cout << "PCID: filled height map has " << pFilled->getEmptyCount() << " empty values, max: " << pFilled->getMax() << ", min: " << pFilled->getMin() << std::endl;

	return pFilled;
}

PC_t::Ptr CPCID::__genePointCloud(const std::shared_ptr<pcl::on_nurbs::FittingSurface> vFit, const ptr<core::CHeightMap> vInput, const ptr<core::CHeightMap> vFilled, const core::SAABB& vBox, int vPointNumberPerPixel) {
	
	core::CHeightMapSampler Sampler;
	std::vector<vec3f> Samples;

	bool r = Sampler.sample(vInput, vFilled, vPointNumberPerPixel, Samples);
	_EARLY_RETURN(!r, "PCID error: sample height map fails.", nullptr);

	PC_t::Ptr pNew(new PC_t);
	for (const vec3f& e : Samples) {
		pNew->emplace_back(core::CSurfaceUtil::sampleSurface(vFit, vec2f { e.x, e.y }));
	}
	std::cout << "PCID: sample " << pNew->size() << " points." << std::endl;

	r = core::PointCloudUtil::calcNormal(pNew, 5);
	_EARLY_RETURN(!r, "PCID error: new cloud calc normal fails.", nullptr);

	for (int i = 0; i < pNew->size(); i++) {
		auto& p = pNew->at(i);
		Eigen::Vector3f Normal { p.normal_x, p.normal_y, p.normal_z };
		Eigen::Vector3f Dist = Normal * Samples[i].z;
		std::cout << "Dist: [" << Normal[0] << ", " << Normal[1] << ", " << Normal[2] << "] * " << Samples[i].z << " = [" << Dist[0] << ", " << Dist[1] << ", " << Dist[2] << "]" << std::endl;
		p.x += Dist[0];
		p.y += Dist[1];
		p.z += Dist[2];
	}

	return pNew;
}

ptr<core::CHeightMap> CPCID::__denoiseHeightMap(const ptr<core::CHeightMap> vHeightMap, uint vThres) {
	if (vHeightMap->getWidth() <= vThres * 2 || vHeightMap->getHeight() <= vThres * 2) {
		return vHeightMap;
	}

	ptr<core::CHeightMap> pHeight(new core::CHeightMap(vHeightMap->getWidth() - 2 * vThres, vHeightMap->getHeight() - 2 * vThres));
	for (uint i = 0; i < pHeight->getWidth(); i++) {
		for (uint k = 0; k < pHeight->getHeight(); k++) {
			pHeight->setValue(i, k, vHeightMap->getValue(i + vThres, k + vThres));
		}
	}

	std::cout << "height map denoise: from [" << vHeightMap->getWidth() << ", " << vHeightMap->getHeight() << "] to [" << pHeight->getWidth() << ", " << pHeight->getHeight() << "]" << std::endl;

	return pHeight;
}

void CPCID::__recoverHeightMap(const ptr<core::CHeightMap> vRawHeightMap, ptr<core::CHeightMap>& vioFilledHeightMap) {
	int Offset = (vioFilledHeightMap->getWidth() - vRawHeightMap->getWidth()) / 2;
	
	for (uint i = 0; i < vRawHeightMap->getWidth(); i++) {
		for (uint k = 0; k < vRawHeightMap->getHeight(); k++) {
			vioFilledHeightMap->setValue(i + Offset, k + Offset, vRawHeightMap->getValue(i, k));
		}
	}

	cv::Mat PartialImage = core::CMapWrapper::castMap2CVMat<float>(vioFilledHeightMap);
	cv::Mat MaskImage = core::CMapWrapper::castMap2CVMat<std::uint8_t>(core::MapUtil::geneMask<float>(vioFilledHeightMap));
	cv::Mat CompleteImage;

	alg::CImageInpainting Inpainter;
	bool r = Inpainter.run(PartialImage, MaskImage, CompleteImage, alg::CV_TEALA);
	_EARLY_RETURN(!r, "PCID error: partial image inpainting fails.", );

	vioFilledHeightMap = std::get<0>(core::CMapWrapper::castCVMat2Map(CompleteImage));
	_EARLY_RETURN(!vioFilledHeightMap->isValid(), "PCID error: cast cv mat 2 complete map fails.", );

	std::cout << "PCID: recover image." << std::endl;
}

bool CPCID::__removeExcessPoints(const PC_t::Ptr vInput, PC_t::Ptr& vioFilled) {
	core::CDuplicateRemover Remover;
	bool r = Remover.run(vInput, vioFilled, 5);
	_EARLY_RETURN(!r, "PCID error: remove excess points fails.", false);

	return true;
}


