#pragma once

#include "PointCloudType.h"
#include "Common.h"
#include <opencv2/opencv.hpp>

class Task {

public:

	static double calcFittingTime(const PC_t::Ptr pInput, int d, int r, int it);
	static bool runPCID(const PC_t::Ptr pInput, const PC_t::Ptr pSub, PC_t::Ptr& voOutput, int workRes, int recoRes, int d, int re, int it);
	static bool runDGI(const PC_t::Ptr pInput, PC_t::Ptr& voOutput, int workRes, int recoRes);
	static bool generateHeightMap(const PC_t::Ptr pInput, const PC_t::Ptr pSub, int workRes, int recoRes, int d, int re, int it);
	static bool generateHeightMap(const PC_t::Ptr pInput, int workRes, int recoRes);
	static void generateHeightMap(const PC_t::Ptr pLarge, const PC_t::Ptr pHole, const PC_t::Ptr pSub, int Res, int d, int re, int it, const std::string& vId);

private:

	static float __calcPC2Surface(const std::vector<vec3f>& vProjs);

};