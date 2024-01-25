#include "pch.h"
#include "HeightMap2PCMapper.h"
#include "HeightMap.h"
#include "MapUtil.h"

using namespace core;

PC_t::Ptr CHeightMap2PCMapper::map2PC(const std::shared_ptr<CHeightMap> vHole, const std::shared_ptr<CHeightMap> vFilled, int vPointNumberPerPixel) {
	PC_t::Ptr pNewCloud(new PC_t);
	_EARLY_RETURN(!vHole->isValid() || !vFilled->isValid(), "mapper error: input maps are invalid.", pNewCloud);
	_EARLY_RETURN(vPointNumberPerPixel <= 0, "mapper error: point number per pixel <= 0.", pNewCloud);
	
	std::vector<vec2f> SampleSequence;
	__geneSamplePoints(vHole, SampleSequence, vPointNumberPerPixel);

	pNewCloud = __geneNewCloud(vFilled, SampleSequence);

	return pNewCloud;
}

void CHeightMap2PCMapper::__geneSamplePoints(const std::shared_ptr<CHeightMap> vHole, std::vector<vec2f>& voSampleSequence, int vPointNumberPerPixel) {
	for (int i = 0; i < vHole->getWidth(); i++) {
		for (int k = 0; k < vHole->getHeight(); k++) {
			if (vHole->isEmpty(i, k)) {
				for (int m = 0; m < vPointNumberPerPixel; m++) {
					float x = MathUtil::geneRandomReal(0.0f, 1.0f);
					float y = MathUtil::geneRandomReal(0.0f, 1.0f);
					voSampleSequence.emplace_back(vec2f { x + (float)i, y + (float)k });
				}
			}
		}
	}

	log("generate sample sequence size: " + std::to_string(voSampleSequence.size()));
}

PC_t::Ptr CHeightMap2PCMapper::__geneNewCloud(const std::shared_ptr<CHeightMap> vFilled, std::vector<vec2f>& vSampleSequence) {
	PC_t::Ptr pFilled(new PC_t);

	for (const auto& e : vSampleSequence) {
		float SampleValue = vFilled->bisample(e.x, e.y);
		pFilled->emplace_back(Point_t(e.x, e.y, SampleValue));
	}

	log("generate sample sequence size: " + std::to_string(pFilled->size()));

	return pFilled;
}
