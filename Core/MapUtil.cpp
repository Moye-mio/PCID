#include "pch.h"
#include "MapUtil.h"

using namespace core;

ptr<CGradientMap> MapUtil::geneGradient(const ptr<CHeightMap> h) {
	_EARLY_RETURN(h->isValid() == false, "generate gradient error: height map is invalid.", nullptr);

	ptr<CGradientMap> pGradient(new CGradientMap(h->getWidth(), h->getHeight(), vec2f { 0, 0 }));
	for (uint i = 0; i < h->getWidth(); i++) {
		for (uint k = 0; k < h->getHeight(); k++) {
			auto r1 = h->calcGradient(i, k, 0);
			auto r2 = h->calcGradient(i, k, 1);
			if (r1.has_value() && r2.has_value()) {
				pGradient->setValue(i, k, vec2f { r1.value(), r2.value() });
			}
			else {
				pGradient->setEmpty(i, k);
			}
		}
	}

	return pGradient;
}

ptr<CGradientMap> MapUtil::geneGradient(const ptr<CGradientMap> g) {
	_EARLY_RETURN(g->isValid() == false, "generate gradient error: gradient map is invalid.", nullptr);

	ptr<CGradientMap> pGradient(new CGradientMap(g->getWidth(), g->getHeight(), vec2f { 0, 0 }));
	ptr<CHeightMap> pHeightX = getHeightMapFromGradientMap(g, 0);
	ptr<CHeightMap> pHeightY = getHeightMapFromGradientMap(g, 1);

	for (uint i = 0; i < g->getWidth(); i++) {
		for (uint k = 0; k < g->getHeight(); k++) {
			auto r1 = pHeightX->calcGradient(i, k, 0);
			auto r2 = pHeightY->calcGradient(i, k, 1);
			if (r1.has_value() && r2.has_value()) {
				pGradient->setValue(i, k, vec2f { r1.value(), r2.value() });
			}
			else {
				pGradient->setEmpty(i, k);
			}
		}
	}

	return pGradient;
}

ptr<CHeightMap> MapUtil::getHeightMapFromGradientMap(const ptr<CGradientMap> g, uint axis) {
	_EARLY_RETURN(g->isValid() == false, "get height map from gradient map error: gradient map is invalid.", nullptr);
	
	ptr<CHeightMap> pHeight(new CHeightMap(g->getWidth(), g->getHeight(), 0.0f));
	for (uint i = 0; i < g->getWidth(); i++) {
		for (uint k = 0; k < g->getHeight(); k++) {
			if (axis) {
				pHeight->setValue(i, k, g->getValue(i, k).x);
			}
			else {
				pHeight->setValue(i, k, g->getValue(i, k).y);
			}
		}
	}

	return pHeight;
}


