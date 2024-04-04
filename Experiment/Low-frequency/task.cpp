#include "task.h"
#include "header.h"

double Task::calcFittingTime(const PC_t::Ptr pInput, int d, int re, int it) {
	auto Time1 = std::chrono::steady_clock::now();

	/* Fitting */
	for (int i = 0; i < 10; i++) {
		core::CNurbsFitting Fitting;
		//Fitting.setCVSavePath("ControlPts/pts.txt");
		//Fitting.setKnotSavePath("ControlPts/knots.txt");
		bool r = Fitting.run(pInput, d, re, it);
	}

	auto Time2 = std::chrono::steady_clock::now();
	double TimeSpan = std::chrono::duration<double, std::milli>(Time2 - Time1).count();
	std::cout << "Fitting use time: " << TimeSpan / 10 << "ms" << std::endl;

	return TimeSpan / 10;
}

bool Task::runPCID(const PC_t::Ptr pInput, const PC_t::Ptr pSub, PC_t::Ptr& voOutput, int workRes, int recoRes, int d, int re, int it) {
	CPCID PCID;
	PCID.setResolution(workRes, recoRes);
	PCID.setFitPara(d, re, it);
	bool r = PCID.run(pInput, pSub, voOutput);

	if (r == false) {
		std::cout << "PCID error." << std::endl;
	}
	else {
		std::cout << "PCID succeeds." << std::endl;
	}

	return r;
}

bool Task::runDGI(const PC_t::Ptr pInput, PC_t::Ptr& voOutput, int workRes, int recoRes) {
	CDGI DGI;
	DGI.setResolution(workRes, recoRes);
	bool r = DGI.run(pInput, voOutput);

	if (r == false) {
		std::cout << "DGI error." << std::endl;
	}
	else {
		std::cout << "DGI succeeds." << std::endl;
	}

	return r;
}

bool Task::generateHeightMap(const PC_t::Ptr pInput, const PC_t::Ptr pSub, int workRes, int recoRes, int d, int re, int it) {
	_EARLY_RETURN(!isPointCloudValid(pInput) || !isPointCloudValid(pSub), "PCID error: input or sub is not valid.", false);

	core::CNurbsFitting Fitting;
	bool r = Fitting.run(pSub, d, re, it);
	const auto Fit = Fitting.getFit();

	std::vector<SProj> Projs;
	r = core::CSurfaceUtil::calcProjPoints(Fit, pInput, Projs);
	_EARLY_RETURN(!r, "PCID error: calc proj points fails.", false);

	core::CProjManager ProjManager;
	r = ProjManager.calcDirection(Projs);
	_EARLY_RETURN(!r, "PCID error: calc proj direction fails.", false);

	std::vector<vec3f> BriefProjs;
	float maxd = -FLT_MAX, mind = FLT_MAX;
	for (const auto& p : Projs) {
		BriefProjs.emplace_back(vec3f { p.uv.x, p.uv.y, p.dist });
		maxd = std::fmaxf(maxd, p.dist);
		mind = std::fminf(mind, p.dist);
	}

	std::cout << "dist from pc to surface: " << __calcPC2Surface(BriefProjs) << std::endl;

	std::vector<vec2i> ProjCoors;
	core::CHeightMapGenerator HMGenerator;
	ptr<core::CHeightMap> pHeight = HMGenerator.generate(BriefProjs, workRes, workRes, true);
	HMGenerator.dumpProjCoor(ProjCoors);

	auto encode = [](int x, int y, int k) -> int { return x * k + y; };
	auto decode = [](int v, int k) -> vec2i { return vec2i { v / k, v % k }; };

	std::unordered_map<int, std::vector<int>> um;
	for (int i = 0; i < ProjCoors.size(); i++) {
		auto& Coor = ProjCoors[i];
		int Key = encode(Coor.x, Coor.y, 1000);
		if (auto it = um.find(Key); it != um.end()) {
			it->second.push_back(i);
		}
		else {
			um.insert(std::make_pair(Key, std::vector<int>(1, i)));
		}
	}

	std::shared_ptr<core::CHeightMap> pSE(new core::CHeightMap(pHeight->getWidth(), pHeight->getHeight()));
	for (auto& e : um) {
		vec2i Coor = decode(e.first, 1000);
		std::vector<int>& Ids = e.second;
		float Ave = 0, se = 0;
		for (auto v : Ids) {
			Ave += BriefProjs[v].z;
		}
		for (auto v : Ids) {
			se += (BriefProjs[v].z - Ave) * (BriefProjs[v].z - Ave);
		}
		pSE->setValue(Coor.x, Coor.y, se / Ids.size());
	}

	float AveSE = 0;
	int Count = 0;
	for (int i = 0; i < pSE->getWidth(); i++) {
		for (int k = 0; k < pSE->getHeight(); k++) {
			if (pSE->isEmpty(i, k) == false) {
				AveSE += pSE->getValue(i, k);
				Count++;
			}
		}
	}
	AveSE /= Count;

	std::cout << "Average SE: " << AveSE << std::endl;

	return r;
}

bool Task::generateHeightMap(const PC_t::Ptr pInput, int workRes, int recoRes) {
	_EARLY_RETURN(!isPointCloudValid(pInput), "DGI error: input or sub is not valid.", false);

	std::vector<vec2i> ProjCoors;
	core::CHeightMapGenerator HMGenerator;
	ptr<core::CHeightMap> pHeight = HMGenerator.generate(pInput, workRes, workRes, true);
	HMGenerator.dumpProjCoor(ProjCoors);

	auto encode = [](int x, int y, int k) -> int { return x * k + y; };
	auto decode = [](int v, int k) -> vec2i { return vec2i { v / k, v % k }; };

	std::unordered_map<int, std::vector<int>> um;
	for (int i = 0; i < ProjCoors.size(); i++) {
		auto& Coor = ProjCoors[i];
		int Key = encode(Coor.x, Coor.y, 1000);
		if (auto it = um.find(Key); it != um.end()) {
			it->second.push_back(i);
		}
		else {
			um.insert(std::make_pair(Key, std::vector<int>(1, i)));
		}
	}

	std::vector<vec3f> BriefProjs;
	for (const auto& p : *pInput) {
		BriefProjs.emplace_back(vec3f { p.x, p.y, p.z });
	}

	std::cout << "dist from pc to surface: " << __calcPC2Surface(BriefProjs) << std::endl;

	std::shared_ptr<core::CHeightMap> pSE(new core::CHeightMap(pHeight->getWidth(), pHeight->getHeight()));
	for (auto& e : um) {
		vec2i Coor = decode(e.first, 1000);
		std::vector<int>& Ids = e.second;
		float Ave = 0, se = 0;
		for (auto v : Ids) {
			Ave += BriefProjs[v].z;
		}
		for (auto v : Ids) {
			se += (BriefProjs[v].z - Ave) * (BriefProjs[v].z - Ave);
		}
		pSE->setValue(Coor.x, Coor.y, se / Ids.size());
	}

	float AveSE = 0;
	int Count = 0;
	for (int i = 0; i < pSE->getWidth(); i++) {
		for (int k = 0; k < pSE->getHeight(); k++) {
			if (pSE->isEmpty(i, k) == false) {
				AveSE += pSE->getValue(i, k);
				Count++;
			}
		}
	}
	AveSE /= Count;

	std::cout << "Average SE: " << AveSE << std::endl;

	return true;
}

void Task::generateHeightMap(const PC_t::Ptr pLarge, const PC_t::Ptr pHole, const PC_t::Ptr pSub, int Res, int d, int re, int it, const std::string& vId) {
	core::CNurbsFitting Fitting;
	Fitting.run(pSub, d, re, it);
	const auto Fit = Fitting.getFit();
	
	std::vector<PC_t::Ptr> Clouds;
	Clouds.push_back(pLarge);
	Clouds.push_back(pHole);

	for (int i = 0; i < Clouds.size(); i++) {
		std::vector<SProj> Projs;
		core::CSurfaceUtil::calcProjPoints(Fit, Clouds[i], Projs);
		core::CProjManager ProjManager;
		ProjManager.calcDirection(Projs);
		std::vector<vec3f> BriefProjs;
		for (const auto& p : Projs) {
			BriefProjs.emplace_back(vec3f { p.uv.x, p.uv.y, p.dist });
		}
		core::CHeightMapGenerator HMGenerator;
		ptr<core::CHeightMap> pHeight = HMGenerator.generate(BriefProjs, Res, Res, false);

		if (i == 0) {
			core::CMapWrapper::saveMapToLocal(pHeight, "Images/HeightMap/" + vId + "/gt.png", true);
		}
		else {
			ptr<core::CMaskMap> pMask = core::MapUtil::geneMask<float>(pHeight);
			cv::Mat Mask = core::CMapWrapper::castMap2CVMat<std::uint8_t>(pMask);
			cv::imwrite("Images/HeightMap/" + vId + "/mask.png", Mask);
		}
	}
}

float Task::__calcPC2Surface(const std::vector<vec3f>& vProjs) {
	float ave = 0.0f;
	for (auto e : vProjs) {
		ave += std::fabsf(e.z);
	}
	return ave / vProjs.size();
}
