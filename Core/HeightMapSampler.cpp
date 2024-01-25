#include "pch.h"
#include "HeightMapSampler.h"
#include "HeightMap.h"
#include "MapUtil.h"

using namespace core;

bool CHeightMapSampler::sample(const std::shared_ptr<CHeightMap> vHole, const std::shared_ptr<CHeightMap> vFilled, int vPointNumberPerPixel, std::vector<vec3f>& voSampless){
	_EARLY_RETURN(!vHole->isValid() || !vFilled->isValid(), "mapper error: input maps are invalid.", false);
	_EARLY_RETURN(vPointNumberPerPixel <= 0, "mapper error: point number per pixel <= 0.", false);
	
	voSampless.clear();
	__geneSamplePoints(vHole, voSampless, vPointNumberPerPixel);
	__sample(vFilled, voSampless);

	log("generate sample sequence size: " + std::to_string(voSampless.size()));

	return true;
}

void CHeightMapSampler::__geneSamplePoints(const std::shared_ptr<CHeightMap> vHole, std::vector<vec3f>& voSampleSequence, int vPointNumberPerPixel) {
	for (int i = 0; i < vHole->getWidth(); i++) {
		for (int k = 0; k < vHole->getHeight(); k++) {
			if (vHole->isEmpty(i, k)) {
				for (int m = 0; m < vPointNumberPerPixel; m++) {
					float x = MathUtil::geneRandomReal(0.0f, 1.0f);
					float y = MathUtil::geneRandomReal(0.0f, 1.0f);
					voSampleSequence.emplace_back(vec3f { x + (float)i, y + (float)k, 0.0f });
				}
			}
		}
	}
}

void CHeightMapSampler::__sample(const std::shared_ptr<CHeightMap> vFilled, std::vector<vec3f>& vioSampleSequence) {
	float Width = vFilled->getWidth();
	float Height = vFilled->getHeight();

	for (auto& e : vioSampleSequence) {
		e.z = vFilled->bisample(e.x, e.y);
		e.x /= Width;
		e.y /= Height;
	}
}
